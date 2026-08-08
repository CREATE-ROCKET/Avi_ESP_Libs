#pragma once

#include <cstddef>
#include <cstdint>

#include "I2CCREATE.h"
#include "SPICREATE.h"
#include "avi_esp_libs/timeout.h"

class LPS25HB {
public:
  enum class Address : uint8_t { low = 0x5C, high = 0x5D };
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
    Odr odr{Odr::hz25};
    PressureAverage pressure_average{PressureAverage::samples512};
    TemperatureAverage temperature_average{TemperatureAverage::samples64};
    avi::Timeout one_shot_timeout{avi::Timeout::milliseconds(100)};
  };
  struct SpiConfig {
    uint32_t frequency_hz{8000000};
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

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const SpiConfig &spi_config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const SpiConfig &spi_config,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(I2CCREATE &i2c, Address address);
  [[nodiscard]] esp_err_t begin(I2CCREATE &i2c, Address address,
                                const Config &config);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t available(bool &ready);
  [[nodiscard]] bool available();
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  enum class Transport : uint8_t { none, spi, i2c };
  [[nodiscard]] esp_err_t configure(const Config &config);
  [[nodiscard]] esp_err_t readRegister(uint8_t address, uint8_t &value);
  [[nodiscard]] esp_err_t writeRegister(uint8_t address, uint8_t value);
  [[nodiscard]] esp_err_t readRegisters(uint8_t address, uint8_t *data,
                                        std::size_t length);
  [[nodiscard]] uint8_t control2Base() const;

  // 利用中は選択したbusが本オブジェクトより長く生存する必要がある。
  SPICREATE *spi_{nullptr};
  I2CCREATE *i2c_{nullptr};
  SPICREATE::Device spi_device_{nullptr};
  I2CCREATE::Device i2c_device_{I2CCREATE::kInvalidDevice};
  Config config_{};
  Transport transport_{Transport::none};
  bool initialized_{false};
};
