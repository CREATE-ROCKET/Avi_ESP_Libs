#pragma once
#include "CANCREATE_lib.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <cstdint>
class CANCREATE {
public:
  CANCREATE() = default;
  ~CANCREATE();
  CANCREATE(const CANCREATE &) = delete;
  CANCREATE &operator=(const CANCREATE &) = delete;
  [[nodiscard]] esp_err_t begin(gpio_num_t tx, gpio_num_t rx,
                                uint32_t bitrate = 500000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t write(const CANCREATEFrame &frame,
                                uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t read(CANCREATEFrame &frame,
                               uint32_t timeout_ms = 100);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  bool initialized_{false};
  void *backend_{nullptr};
};
