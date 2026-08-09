#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "SPICREATE.h"
#include "avi_esp_libs/timeout.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class ICM42688 {
public:
  enum class AccelRange : uint8_t { g2, g4, g8, g16 };
  enum class GyroRange : uint8_t {
    dps15_625,
    dps31_25,
    dps62_5,
    dps125,
    dps250,
    dps500,
    dps1000,
    dps2000
  };
  enum class AccelOdr : uint8_t {
    hz12_5,
    hz25,
    hz50,
    hz100,
    hz200,
    hz500,
    hz1000,
    hz2000,
    hz4000,
    hz8000,
    hz16000,
    hz32000
  };
  enum class GyroOdr : uint8_t {
    hz12_5,
    hz25,
    hz50,
    hz100,
    hz200,
    hz500,
    hz1000,
    hz2000,
    hz4000,
    hz8000,
    hz16000,
    hz32000
  };
  enum class Filter : uint8_t {
    odr_div2,
    odr_div4,
    odr_div5,
    odr_div8,
    odr_div10,
    odr_div16,
    odr_div20,
    odr_div40
  };

  struct FifoConfig {
    bool enabled{false};
    uint16_t watermark_records{1};
  };

  struct Config {
    uint32_t frequency_hz{8000000};
    AccelRange accel_range{AccelRange::g16};
    GyroRange gyro_range{GyroRange::dps2000};
    AccelOdr accel_odr{AccelOdr::hz1000};
    GyroOdr gyro_odr{GyroOdr::hz1000};
    Filter filter{Filter::odr_div4};
    gpio_num_t int_gpio{GPIO_NUM_NC};
    FifoConfig fifo{};
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

  struct FifoRawData {
    std::array<int16_t, 3> acceleration{};
    std::array<int16_t, 3> angular_velocity{};
    int8_t temperature{};
    uint16_t timestamp_ticks{};
    bool acceleration_valid{false};
    bool angular_velocity_valid{false};
    bool temperature_valid{true};
    bool accel_odr_changed{false};
    bool gyro_odr_changed{false};
  };

  struct FifoData {
    std::array<float, 3> acceleration_g{};
    std::array<float, 3> angular_velocity_dps{};
    float temperature_celsius{};
    uint16_t timestamp_ticks{};
    uint64_t timestamp_us{};
    bool acceleration_valid{false};
    bool angular_velocity_valid{false};
    bool temperature_valid{true};
    bool accel_odr_changed{false};
    bool gyro_odr_changed{false};
  };

  struct FifoStatus {
    uint16_t records_available{};
    bool threshold{false};
    bool full{false};
    uint16_t lost_packets{};
    bool faulted{false};
  };

  struct Status {
    bool data_ready{false};
  };

  struct SelfTestResult {
    bool passed{false};
    bool restored{false};
    std::array<bool, 3> accel_passed{};
    std::array<bool, 3> gyro_passed{};
    std::array<int32_t, 3> accel_baseline{};
    std::array<int32_t, 3> accel_stimulated{};
    std::array<int32_t, 3> accel_response{};
    std::array<int32_t, 3> gyro_baseline{};
    std::array<int32_t, 3> gyro_stimulated{};
    std::array<int32_t, 3> gyro_response{};
  };

  ICM42688() = default;
  ~ICM42688();
  ICM42688(const ICM42688 &) = delete;
  ICM42688 &operator=(const ICM42688 &) = delete;
  ICM42688(ICM42688 &&) = delete;
  ICM42688 &operator=(ICM42688 &&) = delete;

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t available(bool &ready);
  [[nodiscard]] bool available();
  [[nodiscard]] esp_err_t
  waitDataReady(avi::Timeout timeout = avi::Timeout::noWait());
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);
  [[nodiscard]] esp_err_t getFifoStatus(FifoStatus &status);
  [[nodiscard]] esp_err_t fifoAvailable(std::size_t &records);
  [[nodiscard]] esp_err_t
  waitFifo(avi::Timeout timeout = avi::Timeout::noWait());
  [[nodiscard]] esp_err_t readFifoRaw(FifoRawData *data, std::size_t capacity,
                                      std::size_t &count);
  [[nodiscard]] esp_err_t readFifo(FifoData *data, std::size_t capacity,
                                   std::size_t &count);
  [[nodiscard]] esp_err_t
  selfTest(SelfTestResult &result,
           avi::Timeout timeout = avi::Timeout::milliseconds(2500));
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  struct InterruptState {
    StaticSemaphore_t storage{};
    SemaphoreHandle_t signal{nullptr};
  };
  static void interruptIsr(void *context);
  [[nodiscard]] esp_err_t readFifoCount(uint16_t &records);
  [[nodiscard]] esp_err_t drainFifo();
  [[nodiscard]] esp_err_t readFifoBytes(std::size_t capacity,
                                        std::size_t &records);
  void resetFifoState();

  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  InterruptState interrupt_{};
  gpio_num_t int_gpio_{GPIO_NUM_NC};
  AccelRange accel_range_{AccelRange::g16};
  GyroRange gyro_range_{GyroRange::dps2000};
  Config config_{};
  std::array<uint8_t, 2080> fifo_buffer_{};
  uint64_t fifo_timestamp_us_{0};
  uint8_t fifo_timestamp_remainder_{0};
  bool fifo_faulted_{false};
  bool initialized_{false};
};
