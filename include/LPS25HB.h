#pragma once

#include <cstdint>

#include "SPICREATE.h"

class LPS25HB {
public:
  enum class Odr : uint8_t { one_shot, hz1, hz7, hz12_5, hz25 };
  enum class PressureAverage : uint8_t {
    samples8,
    samples32,
    samples128,
    samples512
  };
  enum class TemperatureAverage : uint8_t {
    samples8,
    samples16,
    samples32,
    samples64
  };

  struct Config {
    uint32_t frequency_hz{8000000};
    Odr odr{Odr::hz25};
    PressureAverage pressure_average{PressureAverage::samples512};
    TemperatureAverage temperature_average{TemperatureAverage::samples64};
    uint32_t one_shot_timeout_ms{100};
  };

  struct RawData {
    int32_t pressure{0};
    int16_t temperature{0};
  };

  struct Data {
    float pressure_pa{0.0F};
    float temperature_celsius{0.0F};
  };

  struct Status {
    bool pressure_ready{false};
    bool temperature_ready{false};
    bool pressure_overrun{false};
    bool temperature_overrun{false};
  };

  LPS25HB() = default;
  ~LPS25HB();
  LPS25HB(const LPS25HB &) = delete;
  LPS25HB &operator=(const LPS25HB &) = delete;
  LPS25HB(LPS25HB &&) = delete;
  LPS25HB &operator=(LPS25HB &&) = delete;

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency_hz = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t available(bool &ready);
  [[nodiscard]] bool available();
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  // 利用中はSPIバスが本オブジェクトより長く生存する必要がある。
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  Config config_{};
  bool initialized_{false};
};
