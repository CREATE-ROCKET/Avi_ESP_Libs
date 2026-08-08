#pragma once

#include <cstdint>

#include "I2CCREATE.h"

class SSCDRRN005PD2A5 {
public:
  enum class SensorStatus : uint8_t {
    normal,
    command_mode,
    stale,
    diagnostic_fault
  };
  struct RawData {
    uint16_t pressure_counts{};
    uint16_t temperature_counts{};
    SensorStatus status{SensorStatus::normal};
  };
  struct Data {
    float differential_pressure_pa{};
    float temperature_celsius{};
  };

  SSCDRRN005PD2A5() = default;
  ~SSCDRRN005PD2A5();
  SSCDRRN005PD2A5(const SSCDRRN005PD2A5 &) = delete;
  SSCDRRN005PD2A5 &operator=(const SSCDRRN005PD2A5 &) = delete;
  SSCDRRN005PD2A5(SSCDRRN005PD2A5 &&) = delete;
  SSCDRRN005PD2A5 &operator=(SSCDRRN005PD2A5 &&) = delete;

  [[nodiscard]] esp_err_t begin(I2CCREATE &i2c);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);

private:
  I2CCREATE *i2c_{nullptr};
  I2CCREATE::Device device_{I2CCREATE::kInvalidDevice};
  bool initialized_{false};
};
