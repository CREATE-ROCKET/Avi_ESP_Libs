#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "driver/gpio.h"
#include "esp_err.h"

class CANCREATE {
public:
  enum class Bitrate : uint32_t {
    kbps25 = 25'000,
    kbps50 = 50'000,
    kbps100 = 100'000,
    kbps125 = 125'000,
    kbps250 = 250'000,
    kbps500 = 500'000,
    kbps800 = 800'000,
    mbps1 = 1'000'000,
  };
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
    Bitrate bitrate{Bitrate::kbps500};
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
                                Bitrate bitrate = Bitrate::kbps500);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t write(const Frame &frame, uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t write(uint32_t identifier, uint8_t value,
                                uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t write(uint32_t identifier, const uint8_t *data,
                                std::size_t length, uint32_t timeout_ms = 100);
  template <std::size_t N>
  [[nodiscard]] esp_err_t write(uint32_t identifier, const uint8_t (&data)[N],
                                uint32_t timeout_ms = 100) {
    return write(identifier, data, N, timeout_ms);
  }
  template <std::size_t N>
  [[nodiscard]] esp_err_t write(uint32_t identifier,
                                const std::array<uint8_t, N> &data,
                                uint32_t timeout_ms = 100) {
    return write(identifier, data.data(), data.size(), timeout_ms);
  }
  [[nodiscard]] esp_err_t writeText(uint32_t identifier, std::string_view text,
                                    uint32_t timeout_ms = 100);
  [[nodiscard]] esp_err_t read(Frame &frame, uint32_t timeout_ms = 0);
  [[nodiscard]] std::size_t available() const;
  [[nodiscard]] esp_err_t available(std::size_t &count) const;
  [[nodiscard]] esp_err_t getStatus(Status &status) const;
  [[nodiscard]] esp_err_t recover(uint32_t timeout_ms = 1000);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  bool initialized_{false};
  void *backend_{nullptr};
};
