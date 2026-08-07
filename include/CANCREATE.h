#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

class CANCREATE {
public:
  enum class Mode : uint8_t { normal, no_ack, listen_only };
  enum class State : uint8_t { stopped, running, bus_off, recovering };

  struct Frame {
    uint32_t identifier{0};
    uint8_t data_length{0};
    uint8_t data[8]{};
    bool extended{false};
    bool remote{false};
  };

  struct Filter {
    uint32_t identifier{0};
    uint32_t mask{0};
    bool extended{false};
    bool enabled{false};
  };

  struct Config {
    gpio_num_t tx{GPIO_NUM_NC};
    gpio_num_t rx{GPIO_NUM_NC};
    uint32_t bitrate{500000};
    Mode mode{Mode::normal};
    Filter filter{};
    uint16_t rx_queue_depth{8};
  };

  struct Status {
    State state{State::stopped};
    uint32_t pending_tx{0};
    uint32_t pending_rx{0};
    uint32_t tx_error_count{0};
    uint32_t rx_error_count{0};
    uint32_t bus_error_count{0};
    uint32_t dropped_rx_count{0};
  };

  CANCREATE() = default;
  ~CANCREATE();
  CANCREATE(const CANCREATE &) = delete;
  CANCREATE &operator=(const CANCREATE &) = delete;
  CANCREATE(CANCREATE &&) = delete;
  CANCREATE &operator=(CANCREATE &&) = delete;

  [[nodiscard]] esp_err_t begin(const Config &config);
  [[nodiscard]] esp_err_t begin(gpio_num_t tx, gpio_num_t rx,
                                uint32_t bitrate = 500000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t write(const Frame &frame, uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t read(Frame &frame, uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t available(std::size_t &count) const;
  [[nodiscard]] esp_err_t getStatus(Status &status) const;
  [[nodiscard]] esp_err_t recover(uint32_t timeout_ms = 1000);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  bool initialized_{false};
  void *backend_{nullptr};
};
