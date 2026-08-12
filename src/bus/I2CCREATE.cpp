#include "I2CCREATE.h"

#include "../compatibility/timeout_internal.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif

#include <algorithm>
#include <climits>

class I2CCREATE::LockGuard {
public:
  explicit LockGuard(I2CCREATE &bus) : bus_(bus), result_(bus.takeBusLock()) {}
  ~LockGuard() {
    if (result_ == ESP_OK)
      bus_.giveBusLock();
  }
  [[nodiscard]] esp_err_t result() const { return result_; }

private:
  I2CCREATE &bus_;
  esp_err_t result_;
};

namespace {
bool validAddress(uint8_t address) { return address <= 0x7F; }
} // namespace

I2CCREATE::~I2CCREATE() {
  if (initialized_)
    (void)end();
}

esp_err_t I2CCREATE::begin(i2c_port_t port, int sda, int scl,
                           uint32_t frequency_hz) {
  Config config{};
  config.port = port;
  config.sda = sda;
  config.scl = scl;
  config.frequency_hz = frequency_hz;
  return begin(config);
}

esp_err_t I2CCREATE::begin(const Config &config) {
  if (initialized_)
    return ESP_ERR_INVALID_STATE;
  uint64_t operation_ms{};
  TickType_t ignored{};
  if (config.port < I2C_NUM_0 || config.port >= I2C_NUM_MAX ||
      !GPIO_IS_VALID_OUTPUT_GPIO(config.sda) ||
      !GPIO_IS_VALID_OUTPUT_GPIO(config.scl) || config.sda == config.scl ||
      config.frequency_hz == 0 || config.frequency_hz > 1000000 ||
      avi::internal::timeoutToTicks(config.lock_timeout, ignored) != ESP_OK ||
      !config.operation_timeout.isFinite() ||
      !config.operation_timeout.millisecondsValue(operation_ms) ||
      operation_ms == 0 || operation_ms > INT_MAX)
    return ESP_ERR_INVALID_ARG;

  SemaphoreHandle_t lock = xSemaphoreCreateMutex();
  if (lock == nullptr)
    return ESP_ERR_NO_MEM;

  esp_err_t result = ESP_OK;
#if ESP_IDF_VERSION_MAJOR >= 5
  i2c_master_bus_config_t bus_config{};
  bus_config.i2c_port = config.port;
  bus_config.sda_io_num = static_cast<gpio_num_t>(config.sda);
  bus_config.scl_io_num = static_cast<gpio_num_t>(config.scl);
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.glitch_ignore_cnt = 7;
  bus_config.flags.enable_internal_pullup = config.enable_internal_pullups;
  i2c_master_bus_handle_t handle = nullptr;
  result = i2c_new_master_bus(&bus_config, &handle);
  bus_handle_ = handle;
#else
  i2c_config_t bus_config{};
  bus_config.mode = I2C_MODE_MASTER;
  bus_config.sda_io_num = config.sda;
  bus_config.scl_io_num = config.scl;
  bus_config.sda_pullup_en =
      config.enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  bus_config.scl_pullup_en =
      config.enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  bus_config.master.clk_speed = config.frequency_hz;
  result = i2c_param_config(config.port, &bus_config);
  if (result == ESP_OK)
    result = i2c_driver_install(config.port, I2C_MODE_MASTER, 0, 0, 0);
#endif
  if (result != ESP_OK) {
    bus_handle_ = nullptr;
    vSemaphoreDelete(lock);
    return result;
  }

  port_ = config.port;
  frequency_hz_ = config.frequency_hz;
  lock_timeout_ = config.lock_timeout;
  operation_timeout_ = config.operation_timeout;
  bus_lock_ = lock;
  devices_.fill(DeviceSlot{});
  initialized_ = true;
  return ESP_OK;
}

esp_err_t I2CCREATE::end() {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (deviceCount() != 0)
    return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 5
  // SAFETY: handleはi2c_new_master_bus()が生成し、end()成功までbusが所有する。
  const esp_err_t result =
      i2c_del_master_bus(static_cast<i2c_master_bus_handle_t>(bus_handle_));
#else
  const esp_err_t result = i2c_driver_delete(port_);
#endif
  if (result == ESP_OK) {
    vSemaphoreDelete(bus_lock_);
    bus_lock_ = nullptr;
    bus_handle_ = nullptr;
    frequency_hz_ = 0;
    initialized_ = false;
  }
  return result;
}

std::size_t I2CCREATE::deviceCount() const {
  return static_cast<std::size_t>(
      std::count_if(devices_.begin(), devices_.end(),
                    [](const DeviceSlot &entry) { return entry.used; }));
}

I2CCREATE::DeviceSlot *I2CCREATE::slot(Device device) {
  if (device >= devices_.size() || !devices_[device].used)
    return nullptr;
  return &devices_[device];
}

esp_err_t I2CCREATE::takeBusLock() {
  TickType_t ticks{};
  if (!initialized_ || bus_lock_ == nullptr ||
      avi::internal::timeoutToTicks(lock_timeout_, ticks) != ESP_OK)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(bus_lock_, ticks) == pdTRUE)
    return ESP_OK;
  return lock_timeout_.isNoWait() ? ESP_ERR_NOT_FINISHED : ESP_ERR_TIMEOUT;
}

void I2CCREATE::giveBusLock() { (void)xSemaphoreGive(bus_lock_); }

esp_err_t I2CCREATE::addDevice(const DeviceConfig &config, Device &device) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (device != kInvalidDevice || !validAddress(config.address))
    return ESP_ERR_INVALID_ARG;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  auto entry = std::find_if(devices_.begin(), devices_.end(),
                            [](const DeviceSlot &item) { return !item.used; });
  if (entry == devices_.end())
    return ESP_ERR_NO_MEM;
  void *handle = nullptr;
#if ESP_IDF_VERSION_MAJOR >= 5
  i2c_device_config_t device_config{};
  device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  device_config.device_address = config.address;
  device_config.scl_speed_hz = frequency_hz_;
  i2c_master_dev_handle_t native = nullptr;
  const esp_err_t result = i2c_master_bus_add_device(
      static_cast<i2c_master_bus_handle_t>(bus_handle_), &device_config,
      &native);
  if (result != ESP_OK)
    return result;
  handle = native;
#endif
  entry->used = true;
  entry->address = config.address;
  entry->handle = handle;
  device = static_cast<Device>(entry - devices_.begin());
  return ESP_OK;
}

esp_err_t I2CCREATE::removeDevice(Device &device) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  DeviceSlot *entry = slot(device);
  if (entry == nullptr)
    return ESP_ERR_NOT_FOUND;
#if ESP_IDF_VERSION_MAJOR >= 5
  // SAFETY: slotのhandleはadd_device成功からrm_device成功まで有効である。
  const esp_err_t result = i2c_master_bus_rm_device(
      static_cast<i2c_master_dev_handle_t>(entry->handle));
  if (result != ESP_OK)
    return result;
#endif
  *entry = DeviceSlot{};
  device = kInvalidDevice;
  return ESP_OK;
}

esp_err_t I2CCREATE::probe(uint8_t address) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (!validAddress(address))
    return ESP_ERR_INVALID_ARG;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  int timeout_ms{};
  const esp_err_t converted =
      avi::internal::timeoutToIntMilliseconds(operation_timeout_, timeout_ms);
  if (converted != ESP_OK)
    return converted;
#if ESP_IDF_VERSION_MAJOR >= 5
  return i2c_master_probe(static_cast<i2c_master_bus_handle_t>(bus_handle_),
                          address, timeout_ms);
#else
  TickType_t ticks{};
  if (avi::internal::timeoutToTicks(operation_timeout_, ticks) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  i2c_cmd_handle_t command = i2c_cmd_link_create();
  if (command == nullptr)
    return ESP_ERR_NO_MEM;

  esp_err_t result = i2c_master_start(command);
  if (result == ESP_OK) {
    result = i2c_master_write_byte(
        command, static_cast<uint8_t>((address << 1) | I2C_MASTER_WRITE), true);
  }
  if (result == ESP_OK)
    result = i2c_master_stop(command);
  if (result == ESP_OK)
    result = i2c_master_cmd_begin(port_, command, ticks);
  i2c_cmd_link_delete(command);
  return result;
#endif
}

esp_err_t I2CCREATE::write(Device device, const uint8_t *data,
                           std::size_t length) {
  if (data == nullptr || length == 0 || length > INT_MAX)
    return ESP_ERR_INVALID_ARG;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  DeviceSlot *entry = slot(device);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 5
  // SAFETY: DeviceSlotはremoveDevice()成功までnative deviceを保持する。
  int timeout_ms{};
  if (avi::internal::timeoutToIntMilliseconds(operation_timeout_, timeout_ms) !=
      ESP_OK)
    return ESP_ERR_INVALID_ARG;
  return i2c_master_transmit(
      static_cast<i2c_master_dev_handle_t>(entry->handle), data, length,
      timeout_ms);
#else
  TickType_t ticks{};
  if (avi::internal::timeoutToTicks(operation_timeout_, ticks) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  return i2c_master_write_to_device(port_, entry->address, data, length, ticks);
#endif
}

esp_err_t I2CCREATE::read(Device device, uint8_t *data, std::size_t length) {
  if (data == nullptr || length == 0 || length > INT_MAX)
    return ESP_ERR_INVALID_ARG;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  DeviceSlot *entry = slot(device);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 5
  int timeout_ms{};
  if (avi::internal::timeoutToIntMilliseconds(operation_timeout_, timeout_ms) !=
      ESP_OK)
    return ESP_ERR_INVALID_ARG;
  return i2c_master_receive(static_cast<i2c_master_dev_handle_t>(entry->handle),
                            data, length, timeout_ms);
#else
  TickType_t ticks{};
  if (avi::internal::timeoutToTicks(operation_timeout_, ticks) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  return i2c_master_read_from_device(port_, entry->address, data, length,
                                     ticks);
#endif
}

esp_err_t I2CCREATE::writeRead(Device device, const uint8_t *write_data,
                               std::size_t write_length, uint8_t *read_data,
                               std::size_t read_length) {
  if (write_data == nullptr || write_length == 0 || read_data == nullptr ||
      read_length == 0 || write_length > INT_MAX || read_length > INT_MAX)
    return ESP_ERR_INVALID_ARG;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  DeviceSlot *entry = slot(device);
  if (entry == nullptr)
    return ESP_ERR_INVALID_STATE;
#if ESP_IDF_VERSION_MAJOR >= 5
  int timeout_ms{};
  if (avi::internal::timeoutToIntMilliseconds(operation_timeout_, timeout_ms) !=
      ESP_OK)
    return ESP_ERR_INVALID_ARG;
  return i2c_master_transmit_receive(
      static_cast<i2c_master_dev_handle_t>(entry->handle), write_data,
      write_length, read_data, read_length, timeout_ms);
#else
  TickType_t ticks{};
  if (avi::internal::timeoutToTicks(operation_timeout_, ticks) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  return i2c_master_write_read_device(port_, entry->address, write_data,
                                      write_length, read_data, read_length,
                                      ticks);
#endif
}

esp_err_t I2CCREATE::writeRegister(Device device, uint8_t address,
                                   uint8_t value) {
  const uint8_t data[]{address, value};
  return write(device, data, sizeof(data));
}

esp_err_t I2CCREATE::readRegister(Device device, uint8_t address,
                                  uint8_t &value) {
  uint8_t next{};
  const esp_err_t result = writeRead(device, &address, 1, &next, 1);
  if (result == ESP_OK)
    value = next;
  return result;
}

esp_err_t I2CCREATE::readRegisters(Device device, uint8_t address,
                                   uint8_t *data, std::size_t length) {
  return writeRead(device, &address, 1, data, length);
}
