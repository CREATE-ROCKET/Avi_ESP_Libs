#pragma once
#pragma message("TODO: NEC920はTier 2であり、ESP32-S3実機では未検証です")

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <cstddef>
#include <cstdint>
class NEC920 {
public:
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
  [[nodiscard]] esp_err_t sleep();
  [[nodiscard]] esp_err_t wake();
  [[nodiscard]] esp_err_t reboot(uint32_t pulse_ms = 10);

private:
  uart_port_t port_{UART_NUM_MAX};
  gpio_num_t reset_{GPIO_NUM_NC};
  gpio_num_t wakeup_{GPIO_NUM_NC};
  bool initialized_{false};
};
