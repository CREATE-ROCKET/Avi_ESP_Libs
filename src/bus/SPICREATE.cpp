#include "SPICREATE.h"

#include <algorithm>
#include <climits>

#include "driver/gpio.h"

SPICREATE::~SPICREATE() { (void)end(); }

esp_err_t SPICREATE::begin(spi_host_device_t host, int sck, int miso, int mosi,
                           std::size_t max_transfer_size) {
  return begin(Config{host, sck, miso, mosi, max_transfer_size});
}

esp_err_t SPICREATE::begin(const Config &bus) {
  if (initialized_)
    return ESP_ERR_INVALID_STATE;
  if (!GPIO_IS_VALID_OUTPUT_GPIO(bus.sck) ||
      !GPIO_IS_VALID_OUTPUT_GPIO(bus.mosi) || !GPIO_IS_VALID_GPIO(bus.miso) ||
      bus.max_transfer_size == 0 || bus.max_transfer_size > INT_MAX)
    return ESP_ERR_INVALID_ARG;
  spi_bus_config_t config{};
  config.sclk_io_num = bus.sck;
  config.miso_io_num = bus.miso;
  config.mosi_io_num = bus.mosi;
  config.quadwp_io_num = -1;
  config.quadhd_io_num = -1;
  config.max_transfer_sz = static_cast<int>(bus.max_transfer_size);
  const esp_err_t result =
      spi_bus_initialize(bus.host, &config, SPI_DMA_CH_AUTO);
  if (result != ESP_OK)
    return result;
  host_ = bus.host;
  initialized_ = true;
  devices_.fill(nullptr);
  return ESP_OK;
}

esp_err_t SPICREATE::end() {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (deviceCount() != 0)
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = spi_bus_free(host_);
  if (result == ESP_OK)
    initialized_ = false;
  return result;
}

std::size_t SPICREATE::deviceCount() const {
  return static_cast<std::size_t>(
      std::count_if(devices_.begin(), devices_.end(),
                    [](Device device) { return device != nullptr; }));
}

bool SPICREATE::owns(Device device) const {
  return device != nullptr &&
         std::find(devices_.begin(), devices_.end(), device) != devices_.end();
}

esp_err_t SPICREATE::addDevice(const DeviceConfig &device_config,
                               Device &device) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (device != nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!GPIO_IS_VALID_OUTPUT_GPIO(device_config.chip_select) ||
      device_config.frequency_hz == 0 || device_config.frequency_hz > INT_MAX ||
      device_config.mode > 3 || device_config.queue_size == 0)
    return ESP_ERR_INVALID_ARG;
  auto slot = std::find(devices_.begin(), devices_.end(), nullptr);
  if (slot == devices_.end())
    return ESP_ERR_NO_MEM;
  spi_device_interface_config_t local{};
  local.clock_speed_hz = static_cast<int>(device_config.frequency_hz);
  local.mode = device_config.mode;
  local.spics_io_num = device_config.chip_select;
  local.queue_size = device_config.queue_size;
  const esp_err_t result = spi_bus_add_device(host_, &local, &device);
  if (result == ESP_OK)
    *slot = device;
  return result;
}

esp_err_t SPICREATE::removeDevice(Device &device) {
  if (device == nullptr)
    return ESP_ERR_INVALID_ARG;
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  auto slot = std::find(devices_.begin(), devices_.end(), device);
  if (slot == devices_.end())
    return ESP_ERR_NOT_FOUND;
  const esp_err_t result = spi_bus_remove_device(device);
  if (result == ESP_OK) {
    *slot = nullptr;
    device = nullptr;
  }
  return result;
}

esp_err_t SPICREATE::transmit(Device device, spi_transaction_t &transaction) {
  if (!initialized_ || !owns(device))
    return ESP_ERR_INVALID_STATE;
  return spi_device_transmit(device, &transaction);
}

esp_err_t SPICREATE::pollingTransmit(Device device,
                                     spi_transaction_t &transaction) {
  if (!initialized_ || !owns(device))
    return ESP_ERR_INVALID_STATE;
  return spi_device_polling_transmit(device, &transaction);
}

esp_err_t SPICREATE::readRegister(Device device, uint8_t address,
                                  uint8_t &value) {
  spi_transaction_t transaction{};
  transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  transaction.length = 16;
  transaction.tx_data[0] = address;
  const esp_err_t result = pollingTransmit(device, transaction);
  if (result == ESP_OK)
    value = transaction.rx_data[1];
  return result;
}

esp_err_t SPICREATE::writeRegister(Device device, uint8_t address,
                                   uint8_t value) {
  spi_transaction_t transaction{};
  transaction.flags = SPI_TRANS_USE_TXDATA;
  transaction.length = 16;
  transaction.tx_data[0] = address;
  transaction.tx_data[1] = value;
  return pollingTransmit(device, transaction);
}

esp_err_t SPICREATE::read(Device device, uint8_t command, uint8_t *data,
                          std::size_t length) {
  if (data == nullptr || length == 0 || length > SIZE_MAX / 8)
    return ESP_ERR_INVALID_ARG;
  spi_transaction_ext_t transaction{};
  transaction.base.flags = SPI_TRANS_VARIABLE_CMD;
  transaction.base.cmd = command;
  transaction.base.length = length * 8;
  transaction.base.rx_buffer = data;
  transaction.command_bits = 8;
  return pollingTransmit(device, transaction.base);
}

esp_err_t SPICREATE::sendCommand(Device device, uint8_t command) {
  spi_transaction_t transaction{};
  transaction.flags = SPI_TRANS_USE_TXDATA;
  transaction.length = 8;
  transaction.tx_data[0] = command;
  return pollingTransmit(device, transaction);
}
