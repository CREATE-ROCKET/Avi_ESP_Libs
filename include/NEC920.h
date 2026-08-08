#pragma once
#pragma message("TODO: NEC920はTier 2であり、ESP32-S3実機では未検証です")

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <array>
#include <cstddef>
#include <cstdint>
class NEC920 {
public:
  enum class MessageId : uint8_t {
    send = 0x11,
    resend = 0x12,
    no_resend = 0x13,
  };

  struct Packet {
    static constexpr std::size_t kMaximumSize = 254;
    std::array<uint8_t, kMaximumSize> bytes{};
    std::size_t size{0};
  };

  NEC920() = default;
  ~NEC920();
  NEC920(const NEC920 &) = delete;
  NEC920 &operator=(const NEC920 &) = delete;
  [[nodiscard]] esp_err_t begin(uart_port_t port, int baud_rate, gpio_num_t rx,
                                gpio_num_t tx, gpio_num_t reset = GPIO_NUM_NC,
                                gpio_num_t wakeup = GPIO_NUM_NC,
                                gpio_num_t mode = GPIO_NUM_NC);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t read(uint8_t *data, size_t capacity, size_t &received,
                               uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t write(const uint8_t *data, size_t length,
                                uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t send(MessageId message_id, uint8_t message_number,
                               const std::array<uint8_t, 4> &destination,
                               const uint8_t *data, std::size_t length,
                               uint32_t timeout_ms = 100);
  template <std::size_t N>
  [[nodiscard]] esp_err_t send(MessageId message_id, uint8_t message_number,
                               const std::array<uint8_t, 4> &destination,
                               const uint8_t (&data)[N],
                               uint32_t timeout_ms = 100) {
    return send(message_id, message_number, destination, data, N, timeout_ms);
  }
  [[nodiscard]] esp_err_t setRfConfig(uint8_t message_number, uint8_t power,
                                      uint8_t channel, uint8_t band,
                                      uint8_t carrier_sense_mode,
                                      uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t receive(Packet &packet, uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t checkCommandResult(const Packet &packet,
                                             uint8_t message_number,
                                             bool &accepted) const;
  [[nodiscard]] esp_err_t available(std::size_t &count) const;
  [[nodiscard]] std::size_t available() const;
  [[nodiscard]] esp_err_t sleep();
  [[nodiscard]] esp_err_t wake();
  [[nodiscard]] esp_err_t reboot(uint32_t pulse_ms = 10);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  uart_port_t port_{UART_NUM_MAX};
  gpio_num_t reset_{GPIO_NUM_NC};
  gpio_num_t wakeup_{GPIO_NUM_NC};
  bool initialized_{false};
};
