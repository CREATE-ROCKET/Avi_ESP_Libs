#include "NEC920.h"
#include "avi_esp_libs/compatibility.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr uint8_t kHeader0 = 0x0F;
constexpr uint8_t kHeader1 = 0x5A;
constexpr std::size_t kPacketOverhead = 13;
constexpr uint8_t kCommandOk = 0x00;
constexpr uint8_t kCommandNg = 0x01;
} // 名前なし名前空間

NEC920::~NEC920() {
  if (initialized_)
    (void)end();
}
esp_err_t NEC920::begin(uart_port_t port, int baud, gpio_num_t rx,
                        gpio_num_t tx, gpio_num_t reset, gpio_num_t wakeup,
                        gpio_num_t mode) {
  if (initialized_)
    return ESP_ERR_INVALID_STATE;
  if (port < 0 || port >= UART_NUM_MAX || baud <= 0 ||
      !GPIO_IS_VALID_GPIO(rx) || !GPIO_IS_VALID_OUTPUT_GPIO(tx))
    return ESP_ERR_INVALID_ARG;
  uart_config_t config{};
  config.baud_rate = baud;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = AVI_UART_DEFAULT_CLOCK;
  esp_err_t r = uart_param_config(port, &config);
  if (r != ESP_OK)
    return r;
  r = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (r != ESP_OK)
    return r;
  r = uart_driver_install(port, 512, 0, 0, nullptr, 0);
  if (r != ESP_OK)
    return r;
  if (reset != GPIO_NUM_NC) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(reset)) {
      (void)uart_driver_delete(port);
      return ESP_ERR_INVALID_ARG;
    }
    gpio_set_direction(reset, GPIO_MODE_OUTPUT);
    gpio_set_level(reset, 1);
  }
  if (wakeup != GPIO_NUM_NC) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(wakeup)) {
      (void)uart_driver_delete(port);
      return ESP_ERR_INVALID_ARG;
    }
    gpio_set_direction(wakeup, GPIO_MODE_OUTPUT);
    gpio_set_level(wakeup, 1);
  }
  if (mode != GPIO_NUM_NC) {
    if (!GPIO_IS_VALID_GPIO(mode)) {
      (void)uart_driver_delete(port);
      return ESP_ERR_INVALID_ARG;
    }
    gpio_set_direction(mode, GPIO_MODE_INPUT);
  }
  port_ = port;
  reset_ = reset;
  wakeup_ = wakeup;
  initialized_ = true;
  return ESP_OK;
}
esp_err_t NEC920::end() {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  auto r = uart_driver_delete(port_);
  if (r == ESP_OK) {
    initialized_ = false;
    port_ = UART_NUM_MAX;
  }
  return r;
}
esp_err_t NEC920::read(uint8_t *data, size_t capacity, size_t &received,
                       uint32_t timeout) {
  received = 0;
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (!data || capacity == 0)
    return ESP_ERR_INVALID_ARG;
  int n = uart_read_bytes(port_, data, capacity, pdMS_TO_TICKS(timeout));
  if (n < 0)
    return ESP_FAIL;
  received = n;
  return n == 0 ? ESP_ERR_TIMEOUT : ESP_OK;
}
esp_err_t NEC920::write(const uint8_t *data, size_t length, uint32_t timeout) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (!data || length == 0)
    return ESP_ERR_INVALID_ARG;
  int n = uart_write_bytes(port_, data, length);
  if (n < 0 || size_t(n) != length)
    return ESP_FAIL;
  return uart_wait_tx_done(port_, pdMS_TO_TICKS(timeout));
}

esp_err_t NEC920::send(MessageId message_id, uint8_t message_number,
                       const std::array<uint8_t, 4> &destination,
                       const uint8_t *data, std::size_t length,
                       uint32_t timeout_ms) {
  const auto id = static_cast<uint8_t>(message_id);
  if ((id != static_cast<uint8_t>(MessageId::send) &&
       id != static_cast<uint8_t>(MessageId::resend) &&
       id != static_cast<uint8_t>(MessageId::no_resend)) ||
      (data == nullptr && length != 0) ||
      length > Packet::kMaximumSize - kPacketOverhead)
    return length > Packet::kMaximumSize - kPacketOverhead
               ? ESP_ERR_INVALID_SIZE
               : ESP_ERR_INVALID_ARG;

  Packet packet{};
  packet.size = length + kPacketOverhead;
  packet.bytes[0] = kHeader0;
  packet.bytes[1] = kHeader1;
  packet.bytes[2] = static_cast<uint8_t>(packet.size);
  packet.bytes[3] = id;
  packet.bytes[4] = message_number;
  for (std::size_t i = 0; i < destination.size(); ++i)
    packet.bytes[5 + i] = destination[i];
  packet.bytes[9] = 0xFF;
  packet.bytes[10] = 0xFF;
  packet.bytes[11] = 0xFF;
  packet.bytes[12] = 0xFF;
  for (std::size_t i = 0; i < length; ++i)
    packet.bytes[kPacketOverhead + i] = data[i];
  return write(packet.bytes.data(), packet.size, timeout_ms);
}

esp_err_t NEC920::setRfConfig(uint8_t message_number, uint8_t power,
                              uint8_t channel, uint8_t band,
                              uint8_t carrier_sense_mode,
                              uint32_t timeout_ms) {
  const uint8_t parameters[]{0x00, power, channel, band, carrier_sense_mode};
  const std::array<uint8_t, 4> module{0xFF, 0xFF, 0xFF, 0xFF};
  return send(MessageId::no_resend, message_number, module, parameters,
              timeout_ms);
}

esp_err_t NEC920::receive(Packet &packet, uint32_t timeout_ms) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  const int64_t deadline = avi_micros() + int64_t{timeout_ms} * 1000;
  Packet next{};
  std::size_t matched{};
  while (avi_micros() <= deadline) {
    uint8_t byte{};
    const int count = uart_read_bytes(port_, &byte, 1, pdMS_TO_TICKS(1));
    if (count < 0)
      return ESP_FAIL;
    if (count == 0)
      continue;
    if (matched == 0 && byte != kHeader0)
      continue;
    if (matched == 1 && byte != kHeader1) {
      matched = byte == kHeader0 ? 1 : 0;
      continue;
    }
    next.bytes[matched++] = byte;
    if (matched == 3) {
      next.size = next.bytes[2];
      if (next.size < kPacketOverhead || next.size > next.bytes.size())
        return ESP_ERR_INVALID_SIZE;
    }
    if (next.size != 0 && matched == next.size) {
      packet = next;
      return ESP_OK;
    }
  }
  return ESP_ERR_TIMEOUT;
}

esp_err_t NEC920::checkCommandResult(const Packet &packet,
                                     uint8_t message_number,
                                     bool &accepted) const {
  if (packet.size < 5 || packet.size > packet.bytes.size() ||
      packet.bytes[0] != kHeader0 || packet.bytes[1] != kHeader1 ||
      packet.bytes[2] != packet.size || packet.bytes[4] != message_number ||
      (packet.bytes[3] != kCommandOk && packet.bytes[3] != kCommandNg))
    return ESP_ERR_INVALID_RESPONSE;
  accepted = packet.bytes[3] == kCommandOk;
  return ESP_OK;
}

esp_err_t NEC920::available(std::size_t &count) const {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  size_t buffered{};
  const esp_err_t result = uart_get_buffered_data_len(port_, &buffered);
  if (result == ESP_OK)
    count = buffered;
  return result;
}

std::size_t NEC920::available() const {
  std::size_t count{};
  return available(count) == ESP_OK ? count : 0;
}
esp_err_t NEC920::sleep() {
  if (!initialized_ || wakeup_ == GPIO_NUM_NC)
    return ESP_ERR_INVALID_STATE;
  return gpio_set_level(wakeup_, 0);
}
esp_err_t NEC920::wake() {
  if (!initialized_ || wakeup_ == GPIO_NUM_NC)
    return ESP_ERR_INVALID_STATE;
  return gpio_set_level(wakeup_, 1);
}
esp_err_t NEC920::reboot(uint32_t pulse) {
  if (!initialized_ || reset_ == GPIO_NUM_NC || pulse == 0)
    return ESP_ERR_INVALID_ARG;
  auto r = gpio_set_level(reset_, 0);
  if (r != ESP_OK)
    return r;
  avi_delay_ms(pulse);
  return gpio_set_level(reset_, 1);
}
