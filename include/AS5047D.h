#pragma once

#include <cstdint>

#include "SPICREATE.h"

class AS5047D {
public:
  enum class AngleSource : uint8_t { compensated, uncompensated };
  struct Config {
    uint32_t frequency_hz{8000000};
    AngleSource angle_source{AngleSource::compensated};
  };
  struct RawData {
    uint16_t angle{0};
  };
  struct Data {
    uint16_t angle_raw{0};
    float angle_degrees{0.0F};
    float angle_radians{0.0F};
  };
  struct ErrorFlags {
    bool parity_error{false};
    bool invalid_command{false};
    bool framing_error{false};
  };
  struct Status {
    bool magnetic_too_low{false};
    bool magnetic_too_high{false};
    bool cordic_overflow{false};
    bool offset_compensation_finished{false};
    uint8_t agc{0};
    uint16_t magnitude{0};
  };

  AS5047D() = default;
  ~AS5047D();
  AS5047D(const AS5047D &) = delete;
  AS5047D &operator=(const AS5047D &) = delete;
  AS5047D(AS5047D &&) = delete;
  AS5047D &operator=(AS5047D &&) = delete;

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);
  [[nodiscard]] esp_err_t startPipelinedRead();
  [[nodiscard]] esp_err_t readPipelinedRaw(RawData &data);
  [[nodiscard]] esp_err_t readPipelined(Data &data);
  [[nodiscard]] esp_err_t stopPipelinedRead();
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t readAndClearErrorFlags(ErrorFlags &flags);
  [[nodiscard]] ErrorFlags lastErrorFlags() const { return last_error_flags_; }
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] bool pipelinedReadActive() const { return pipeline_active_; }

private:
  [[nodiscard]] esp_err_t transferFrame(uint16_t tx, uint16_t &rx);
  [[nodiscard]] esp_err_t readRegister(uint16_t address, uint16_t &value);
  [[nodiscard]] esp_err_t readErrorFlagsInternal(ErrorFlags &flags);
  [[nodiscard]] esp_err_t handleErrorFlag();
  [[nodiscard]] uint16_t angleReadCommand() const;

  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  Config config_{};
  ErrorFlags last_error_flags_{};
  bool pipeline_active_{false};
  bool initialized_{false};
};
