#include "STSCREATE.h"

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"

#include <algorithm>
#include <array>
#include <climits>

class STSCREATE::LockGuard {
public:
  explicit LockGuard(STSCREATE &bus) : bus_(bus), result_(bus.takeBusLock()) {}
  ~LockGuard() {
    if (result_ == ESP_OK)
      bus_.giveBusLock();
  }
  [[nodiscard]] esp_err_t result() const { return result_; }

private:
  STSCREATE &bus_;
  esp_err_t result_;
};

namespace {
constexpr uint8_t kHeader = 0xFF;
constexpr uint8_t kBroadcastId = 0xFE;
constexpr std::size_t kMaximumParameters = 253;
constexpr std::size_t kMaximumPacket = 259;

constexpr uint64_t wireTimeMicroseconds(std::size_t bytes, uint32_t baudrate) {
  return (bytes * 10ULL * 1000000ULL + baudrate - 1) / baudrate;
}

constexpr bool validResponseWait(uint8_t id, bool wait_response) {
  return id != kBroadcastId || !wait_response;
}

constexpr uint8_t checksum(uint8_t id, uint8_t length, uint8_t instruction,
                           const uint8_t *parameters,
                           std::size_t parameter_count) {
  uint16_t sum = id + length + instruction;
  for (std::size_t i = 0; i < parameter_count; ++i)
    sum += parameters[i];
  return static_cast<uint8_t>(~sum);
}

template <std::size_t N>
constexpr bool validResponse(const std::array<uint8_t, N> &bytes) {
  if (N < 6 || bytes[0] != kHeader || bytes[1] != kHeader || bytes[3] != N - 4)
    return false;
  return checksum(bytes[2], bytes[3], bytes[4], bytes.data() + 5, N - 6) ==
         bytes[N - 1];
}

template <std::size_t N>
constexpr std::array<uint8_t, N + 6>
packet(uint8_t id, uint8_t instruction,
       const std::array<uint8_t, N> &parameters) {
  std::array<uint8_t, N + 6> output{};
  output[0] = 0xFF;
  output[1] = 0xFF;
  output[2] = id;
  output[3] = static_cast<uint8_t>(N + 2);
  output[4] = instruction;
  for (std::size_t i = 0; i < N; ++i)
    output[5 + i] = parameters[i];
  output[N + 5] = checksum(id, output[3], instruction, parameters.data(), N);
  return output;
}

template <std::size_t N>
constexpr bool equal(const std::array<uint8_t, N> &left,
                     const std::array<uint8_t, N> &right) {
  for (std::size_t i = 0; i < N; ++i) {
    if (left[i] != right[i])
      return false;
  }
  return true;
}

static_assert(equal(packet(1, 0x01, std::array<uint8_t, 0>{}),
                    std::array<uint8_t, 6>{0xFF, 0xFF, 0x01, 0x02, 0x01,
                                           0xFB}));
static_assert(equal(packet(1, 0x02, std::array<uint8_t, 2>{0x38, 0x02}),
                    std::array<uint8_t, 8>{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38,
                                           0x02, 0xBE}));
static_assert(equal(
    packet(1, 0x03,
           std::array<uint8_t, 7>{0x2A, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}),
    std::array<uint8_t, 13>{0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A, 0x00, 0x08,
                            0x00, 0x00, 0xE8, 0x03, 0xD5}));
static_assert(equal(packet(0xFE, 0x82,
                           std::array<uint8_t, 4>{0x38, 0x08, 0x01, 0x02}),
                    std::array<uint8_t, 10>{0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38,
                                            0x08, 0x01, 0x02, 0x36}));
static_assert(validResponse(std::array<uint8_t, 6>{0xFF, 0xFF, 0x01, 0x02, 0x00,
                                                   0xFC}));
static_assert(validResponse(std::array<uint8_t, 6>{0xFF, 0xFF, 0x01, 0x02, 0x20,
                                                   0xDC}));
static_assert(!validResponse(std::array<uint8_t, 6>{0xFF, 0xFF, 0x01, 0x02,
                                                    0x20, 0xDD}));
static_assert(validResponseWait(1, true));
static_assert(validResponseWait(1, false));
static_assert(validResponseWait(kBroadcastId, false));
static_assert(!validResponseWait(kBroadcastId, true));
static_assert(wireTimeMicroseconds(259, 38400) == 67448);
static_assert((wireTimeMicroseconds(259, 38400) + 999) / 1000 == 68);
static_assert(100 > (wireTimeMicroseconds(259, 38400) + 999) / 1000);

bool validBaudrate(STSCREATE::Baudrate baudrate) {
  switch (baudrate) {
  case STSCREATE::Baudrate::bps1000000:
  case STSCREATE::Baudrate::bps500000:
  case STSCREATE::Baudrate::bps250000:
  case STSCREATE::Baudrate::bps128000:
  case STSCREATE::Baudrate::bps115200:
  case STSCREATE::Baudrate::bps76800:
  case STSCREATE::Baudrate::bps57600:
  case STSCREATE::Baudrate::bps38400:
    return true;
  }
  return false;
}
} // namespace

STSCREATE::~STSCREATE() {
  if (initialized_)
    (void)end();
}

bool STSCREATE::validId(uint8_t id, bool allow_broadcast) {
  return id <= 253 || (allow_broadcast && id == kBroadcastId);
}

esp_err_t STSCREATE::begin(const Config &config) {
  if (initialized_)
    return ESP_ERR_INVALID_STATE;
  uint64_t tx_ms{};
  uint64_t response_ms{};
  TickType_t ignored{};
  avi::internal::Deadline ignored_deadline{};
  if (config.port < UART_NUM_0 || config.port >= UART_NUM_MAX ||
      !GPIO_IS_VALID_OUTPUT_GPIO(config.tx) || !GPIO_IS_VALID_GPIO(config.rx) ||
      config.tx == config.rx ||
      (config.direction_enable != GPIO_NUM_NC &&
       !GPIO_IS_VALID_OUTPUT_GPIO(config.direction_enable)) ||
      !validBaudrate(config.baudrate) ||
      avi::internal::timeoutToTicks(config.lock_timeout, ignored) != ESP_OK ||
      !config.tx_timeout.isFinite() ||
      !config.tx_timeout.millisecondsValue(tx_ms) || tx_ms == 0 ||
      avi::internal::timeoutToTicks(config.tx_timeout, ignored) != ESP_OK ||
      !config.response_timeout.isFinite() ||
      !config.response_timeout.millisecondsValue(response_ms) ||
      response_ms == 0 ||
      avi::internal::makeDeadline(config.response_timeout, ignored_deadline) !=
          ESP_OK)
    return ESP_ERR_INVALID_ARG;
  SemaphoreHandle_t lock = xSemaphoreCreateMutex();
  if (lock == nullptr)
    return ESP_ERR_NO_MEM;
  uart_config_t uart_config{};
  uart_config.baud_rate = static_cast<int>(config.baudrate);
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = AVI_UART_DEFAULT_CLOCK;
  esp_err_t result = uart_param_config(config.port, &uart_config);
  if (result == ESP_OK)
    result = uart_set_pin(config.port, config.tx, config.rx, UART_PIN_NO_CHANGE,
                          UART_PIN_NO_CHANGE);
  if (result == ESP_OK)
    result = uart_driver_install(config.port, 512, 0, 0, nullptr, 0);
  if (result != ESP_OK) {
    vSemaphoreDelete(lock);
    return result;
  }
  port_ = config.port;
  direction_enable_ = config.direction_enable;
  direction_polarity_ = config.direction_polarity;
  lock_timeout_ = config.lock_timeout;
  tx_timeout_ = config.tx_timeout;
  response_timeout_ = config.response_timeout;
  bus_lock_ = lock;
  initialized_ = true;
  if (direction_enable_ != GPIO_NUM_NC) {
    gpio_config_t gpio{};
    gpio.pin_bit_mask = uint64_t{1} << direction_enable_;
    gpio.mode = GPIO_MODE_OUTPUT;
    result = gpio_config(&gpio);
    if (result == ESP_OK)
      result = setTransmitDirection(false);
    if (result != ESP_OK) {
      (void)uart_driver_delete(port_);
      vSemaphoreDelete(bus_lock_);
      bus_lock_ = nullptr;
      initialized_ = false;
      return result;
    }
  }
  return ESP_OK;
}

esp_err_t STSCREATE::end() {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = uart_driver_delete(port_);
  if (result == ESP_OK) {
    vSemaphoreDelete(bus_lock_);
    bus_lock_ = nullptr;
    initialized_ = false;
  }
  return result;
}

esp_err_t STSCREATE::takeBusLock() {
  TickType_t ticks{};
  if (!initialized_ || bus_lock_ == nullptr ||
      avi::internal::timeoutToTicks(lock_timeout_, ticks) != ESP_OK)
    return ESP_ERR_INVALID_STATE;
  if (xSemaphoreTake(bus_lock_, ticks) == pdTRUE)
    return ESP_OK;
  return lock_timeout_.isNoWait() ? ESP_ERR_NOT_FINISHED : ESP_ERR_TIMEOUT;
}

void STSCREATE::giveBusLock() { (void)xSemaphoreGive(bus_lock_); }

esp_err_t STSCREATE::setTransmitDirection(bool transmit) {
  if (direction_enable_ == GPIO_NUM_NC)
    return ESP_OK;
  const bool high =
      direction_polarity_ == DirectionPolarity::tx_high ? transmit : !transmit;
  return gpio_set_level(static_cast<gpio_num_t>(direction_enable_),
                        high ? 1 : 0);
}

esp_err_t STSCREATE::sendPacket(uint8_t id, Instruction instruction,
                                const uint8_t *parameters,
                                std::size_t parameter_count) {
  if (parameter_count > kMaximumParameters ||
      (parameter_count != 0 && parameters == nullptr))
    return ESP_ERR_INVALID_SIZE;
  std::array<uint8_t, kMaximumPacket> bytes{};
  bytes[0] = kHeader;
  bytes[1] = kHeader;
  bytes[2] = id;
  bytes[3] = static_cast<uint8_t>(parameter_count + 2);
  bytes[4] = static_cast<uint8_t>(instruction);
  if (parameter_count != 0)
    std::copy_n(parameters, parameter_count, bytes.begin() + 5);
  bytes[parameter_count + 5] =
      checksum(id, bytes[3], bytes[4], parameters, parameter_count);
  const std::size_t packet_length = parameter_count + 6;
  esp_err_t result = setTransmitDirection(true);
  if (result != ESP_OK)
    return result;
  const int written = uart_write_bytes(port_, bytes.data(), packet_length);
  if (written != static_cast<int>(packet_length))
    result = written < 0 ? ESP_FAIL : ESP_ERR_INVALID_SIZE;
  TickType_t ticks{};
  if (result == ESP_OK) {
    result = avi::internal::timeoutToTicks(tx_timeout_, ticks);
    if (result == ESP_OK)
      result = uart_wait_tx_done(port_, ticks);
  }
  const esp_err_t direction = setTransmitDirection(false);
  return result == ESP_OK ? direction : result;
}

esp_err_t STSCREATE::receivePacket(uint8_t expected_id, uint8_t *data,
                                   std::size_t expected_length,
                                   uint8_t *device_error) {
  if (expected_length != 0 && data == nullptr)
    return ESP_ERR_INVALID_ARG;
  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(response_timeout_, deadline) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  uint8_t byte{};
  const auto readByte = [this, &deadline](uint8_t &value) {
    while (!avi::internal::expired(deadline)) {
      const int count = uart_read_bytes(port_, &value, 1, 1);
      if (count == 1)
        return ESP_OK;
      if (count < 0)
        return ESP_FAIL;
    }
    return ESP_ERR_TIMEOUT;
  };
  unsigned header_count = 0;
  while (header_count < 2) {
    const esp_err_t result = readByte(byte);
    if (result != ESP_OK)
      return result;
    header_count = byte == kHeader ? header_count + 1 : 0;
  }
  uint8_t prefix[3]{};
  esp_err_t result = readByte(prefix[0]);
  while (result == ESP_OK && prefix[0] == kHeader)
    result = readByte(prefix[0]);
  if (result != ESP_OK)
    return result;
  for (std::size_t i = 1; i < 3; ++i) {
    result = readByte(prefix[i]);
    if (result != ESP_OK)
      return result;
  }
  const uint8_t id = prefix[0];
  const uint8_t length = prefix[1];
  const uint8_t error = prefix[2];
  if (id != expected_id || length < 2 ||
      static_cast<std::size_t>(length - 2) != expected_length)
    return ESP_ERR_INVALID_RESPONSE;
  std::array<uint8_t, kMaximumParameters + 1> payload{};
  for (std::size_t i = 0; i < expected_length + 1; ++i) {
    const esp_err_t result = readByte(payload[i]);
    if (result != ESP_OK)
      return result;
  }
  if (checksum(id, length, error, payload.data(), expected_length) !=
      payload[expected_length])
    return ESP_ERR_INVALID_RESPONSE;
  if (device_error != nullptr)
    *device_error = error;
  if (expected_length != 0)
    std::copy_n(payload.begin(), expected_length, data);
  return ESP_OK;
}

esp_err_t STSCREATE::transaction(uint8_t id, Instruction instruction,
                                 const uint8_t *parameters,
                                 std::size_t parameter_count,
                                 bool wait_response, uint8_t *response_data,
                                 std::size_t response_length,
                                 uint8_t *device_error, bool allow_broadcast) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (!validId(id, allow_broadcast) || !validResponseWait(id, wait_response))
    return ESP_ERR_INVALID_ARG;
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  (void)uart_flush_input(port_);
  esp_err_t result = sendPacket(id, instruction, parameters, parameter_count);
  if (result == ESP_OK && wait_response)
    result = receivePacket(id, response_data, response_length, device_error);
  return result;
}

esp_err_t STSCREATE::ping(uint8_t id, uint8_t *device_error) {
  if (id == kBroadcastId)
    return ESP_ERR_INVALID_ARG;
  return transaction(id, Instruction::ping, nullptr, 0, true, nullptr, 0,
                     device_error, false);
}

esp_err_t STSCREATE::read(uint8_t id, uint8_t address, uint8_t *data,
                          std::size_t length, uint8_t *device_error) {
  if (length == 0 || length > 253)
    return ESP_ERR_INVALID_SIZE;
  const uint8_t parameters[]{address, static_cast<uint8_t>(length)};
  return transaction(id, Instruction::read, parameters, sizeof(parameters),
                     true, data, length, device_error, false);
}

esp_err_t STSCREATE::write(uint8_t id, uint8_t address, const uint8_t *data,
                           std::size_t length, bool wait_response,
                           uint8_t *device_error) {
  if (data == nullptr || length == 0 || length > 252)
    return ESP_ERR_INVALID_SIZE;
  std::array<uint8_t, kMaximumParameters> parameters{};
  parameters[0] = address;
  std::copy_n(data, length, parameters.begin() + 1);
  return transaction(id, Instruction::write, parameters.data(), length + 1,
                     wait_response, nullptr, 0, device_error, false);
}

esp_err_t STSCREATE::regWrite(uint8_t id, uint8_t address, const uint8_t *data,
                              std::size_t length, bool wait_response,
                              uint8_t *device_error) {
  if (data == nullptr || length == 0 || length > 252)
    return ESP_ERR_INVALID_SIZE;
  std::array<uint8_t, kMaximumParameters> parameters{};
  parameters[0] = address;
  std::copy_n(data, length, parameters.begin() + 1);
  return transaction(id, Instruction::reg_write, parameters.data(), length + 1,
                     wait_response, nullptr, 0, device_error, false);
}

esp_err_t STSCREATE::action(uint8_t id, bool wait_response,
                            uint8_t *device_error) {
  return transaction(id, Instruction::action, nullptr, 0, wait_response,
                     nullptr, 0, device_error, true);
}

esp_err_t STSCREATE::syncRead(uint8_t address, uint8_t length,
                              const uint8_t *ids, std::size_t id_count,
                              uint8_t *data, std::size_t data_size,
                              uint8_t *device_errors) {
  if (ids == nullptr || data == nullptr || id_count == 0 || length == 0 ||
      id_count > 251 || data_size != id_count * length)
    return ESP_ERR_INVALID_ARG;
  std::array<uint8_t, kMaximumParameters> parameters{};
  parameters[0] = address;
  parameters[1] = length;
  for (std::size_t i = 0; i < id_count; ++i) {
    if (!validId(ids[i], false))
      return ESP_ERR_INVALID_ARG;
    parameters[i + 2] = ids[i];
  }
  LockGuard lock(*this);
  if (lock.result() != ESP_OK)
    return lock.result();
  (void)uart_flush_input(port_);
  esp_err_t result = sendPacket(kBroadcastId, Instruction::sync_read,
                                parameters.data(), id_count + 2);
  for (std::size_t i = 0; i < id_count && result == ESP_OK; ++i)
    result =
        receivePacket(ids[i], data + i * length, length,
                      device_errors == nullptr ? nullptr : &device_errors[i]);
  return result;
}

esp_err_t STSCREATE::syncWrite(uint8_t address, uint8_t length,
                               const uint8_t *ids, std::size_t id_count,
                               const uint8_t *data, std::size_t data_size) {
  if (ids == nullptr || data == nullptr || id_count == 0 || length == 0 ||
      data_size != id_count * length || 2 + id_count * (length + 1) > 253)
    return ESP_ERR_INVALID_ARG;
  std::array<uint8_t, kMaximumParameters> parameters{};
  parameters[0] = address;
  parameters[1] = length;
  std::size_t cursor = 2;
  for (std::size_t i = 0; i < id_count; ++i) {
    if (!validId(ids[i], false))
      return ESP_ERR_INVALID_ARG;
    parameters[cursor++] = ids[i];
    std::copy_n(data + i * length, length, parameters.begin() + cursor);
    cursor += length;
  }
  return transaction(kBroadcastId, Instruction::sync_write, parameters.data(),
                     cursor, false, nullptr, 0, nullptr, true);
}

esp_err_t STSCREATE::recovery(uint8_t id, uint8_t *device_error,
                              bool wait_response) {
  return transaction(id, Instruction::recovery, nullptr, 0, wait_response,
                     nullptr, 0, device_error, false);
}

esp_err_t STSCREATE::resetState(uint8_t id, uint8_t *device_error,
                                bool wait_response) {
  return transaction(id, Instruction::reset, nullptr, 0, wait_response, nullptr,
                     0, device_error, false);
}
