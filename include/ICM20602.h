#pragma once

#include <array>
#include <cstdint>

#include "SPICREATE.h"

class ICM20602 {
public:
  enum class AccelRange : uint8_t { g2, g4, g8, g16 };
  enum class GyroRange : uint8_t { dps250, dps500, dps1000, dps2000 };
  enum class GyroDlpf : uint8_t {
    hz250,
    hz176,
    hz92,
    hz41,
    hz20,
    hz10,
    hz5,
    hz3281
  };
  enum class AccelDlpf : uint8_t {
    hz1046,
    hz218,
    hz99,
    hz44_8,
    hz21_2,
    hz10_2,
    hz5_1,
    hz420
  };

  struct Config {
    uint32_t frequency_hz{8000000};
    AccelRange accel_range{AccelRange::g16};
    GyroRange gyro_range{GyroRange::dps2000};
    AccelDlpf accel_dlpf{AccelDlpf::hz99};
    GyroDlpf gyro_dlpf{GyroDlpf::hz92};
    uint8_t sample_rate_divider{0};
  };

  struct RawData {
    std::array<int16_t, 3> acceleration{};
    std::array<int16_t, 3> angular_velocity{};
    int16_t temperature{};
  };

  struct Data {
    std::array<float, 3> acceleration_g{};
    std::array<float, 3> angular_velocity_dps{};
    float temperature_celsius{};
  };

  struct Status {
    bool data_ready{false};
  };

  ICM20602() = default;
  ~ICM20602();
  ICM20602(const ICM20602 &) = delete;
  ICM20602 &operator=(const ICM20602 &) = delete;
  ICM20602(ICM20602 &&) = delete;
  ICM20602 &operator=(ICM20602 &&) = delete;

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency_hz = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t available(bool &ready);
  [[nodiscard]] bool available();
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);

private:
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  AccelRange accel_range_{AccelRange::g16};
  GyroRange gyro_range_{GyroRange::dps2000};
  bool initialized_{false};
};
