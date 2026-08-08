#pragma once

#include <array>
#include <cstdint>

#include "SPICREATE.h"
#include "avi_esp_libs/timeout.h"

class ICM20948 {
public:
  enum class AccelRange : uint8_t { g2, g4, g8, g16 };
  enum class GyroRange : uint8_t { dps250, dps500, dps1000, dps2000 };
  enum class Dlpf : uint8_t {
    level0,
    level1,
    level2,
    level3,
    level4,
    level5,
    level6,
    level7,
    bypass = 0xFF
  };
  enum class MagnetometerOdr : uint8_t {
    off = 0x00,
    hz10 = 0x02,
    hz20 = 0x04,
    hz50 = 0x06,
    hz100 = 0x08
  };

  struct Config {
    uint32_t frequency_hz{7000000};
    AccelRange accel_range{AccelRange::g16};
    GyroRange gyro_range{GyroRange::dps2000};
    // DLPF有効時はODRが1125 Hz / (1 + divider)となる。
    uint16_t accel_sample_rate_divider{9};
    uint8_t gyro_sample_rate_divider{9};
    // level0からlevel7はデータシートのDLPFCFG値0から7に対応する。
    Dlpf accel_dlpf{Dlpf::level3};
    Dlpf gyro_dlpf{Dlpf::level3};
    MagnetometerOdr magnetometer_odr{MagnetometerOdr::hz100};
    avi::Timeout operation_timeout{avi::Timeout::milliseconds(300)};
  };

  struct RawData {
    std::array<int16_t, 3> acceleration{};
    std::array<int16_t, 3> angular_velocity{};
    int16_t temperature{};
    std::array<int16_t, 3> magnetic{};
    bool magnetic_valid{false};
  };

  struct Data {
    std::array<float, 3> acceleration_g{};
    std::array<float, 3> angular_velocity_dps{};
    float temperature_celsius{};
    std::array<float, 3> magnetic_ut{};
    bool magnetic_valid{false};
  };

  struct Status {
    bool data_ready{false};
    bool magnetometer_enabled{false};
    bool magnetometer_ready{false};
    bool magnetometer_overrun{false};
    bool magnetometer_overflow{false};
    bool auxiliary_i2c_error{false};
  };

  ICM20948() = default;
  ~ICM20948();
  ICM20948(const ICM20948 &) = delete;
  ICM20948 &operator=(const ICM20948 &) = delete;
  ICM20948(ICM20948 &&) = delete;
  ICM20948 &operator=(ICM20948 &&) = delete;

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency_hz = 7000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t available(bool &ready);
  [[nodiscard]] bool available();
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  [[nodiscard]] esp_err_t selectBank(uint8_t bank);
  [[nodiscard]] esp_err_t magnetometerTransfer(uint8_t address, bool read,
                                               uint8_t *value,
                                               avi::Timeout timeout);
  [[nodiscard]] esp_err_t configureMagnetometer(MagnetometerOdr odr);
  [[nodiscard]] esp_err_t shutdownHardware(bool stop_magnetometer);

  // 利用中はSPIバスが本オブジェクトより長く生存する必要がある。
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  Config config_{};
  bool initialized_{false};
};
