#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "STSCREATE.h"

class STS3215 {
public:
  enum class OperatingMode : uint8_t {
    position = 0,
    velocity = 1,
    pwm = 2,
    step = 3
  };
  enum class Persistence : uint8_t { volatile_only, persistent };
  enum class Direction : uint8_t { normal, reverse };
  enum class FeedbackMode : uint8_t { single_turn, multi_turn };
  enum class SpeedUnitMode : uint8_t { steps50_per_second, step_per_second };

  class TorqueLimit {
  public:
    [[nodiscard]] static TorqueLimit raw(uint16_t value);
    [[nodiscard]] static TorqueLimit percent(float value);
    [[nodiscard]] bool valid() const { return valid_; }
    [[nodiscard]] uint16_t rawValue() const { return value_; }
    [[nodiscard]] float percentValue() const { return value_ / 10.0F; }

  private:
    TorqueLimit(uint16_t value, bool valid) : value_(value), valid_(valid) {}
    uint16_t value_{};
    bool valid_{true};
  };

  struct HoldConfig {
    TorqueLimit torque_limit{TorqueLimit::raw(300)};
  };
  struct Motion {
    float speed_deg_s{};
    float acceleration_deg_s2{};
    std::optional<TorqueLimit> torque_limit{};
  };
  struct StallProtection {
    bool enabled{};
    float trigger_torque_percent{};
    uint16_t trigger_time_ms{};
    float protected_torque_percent{};
  };
  struct Status {
    bool overload{};
    bool overcurrent{};
    bool overtemperature{};
    bool encoder_fault{};
    bool voltage_fault{};
    uint8_t raw{};
  };
  struct RawData {
    uint16_t position{};
    uint16_t speed{};
    uint16_t load{};
    uint8_t voltage{};
    uint8_t temperature{};
    bool registered_instruction{};
    uint8_t status{};
    bool moving{};
    uint16_t current{};
  };
  struct Data {
    float position_deg{};
    float speed_deg_s{};
    int16_t load_raw{};
    float voltage_v{};
    float temperature_celsius{};
    float current_a{};
    bool moving{};
    Status status{};
  };
  enum class Register : uint8_t {
    id = 0x05,
    baud_rate = 0x06,
    response_status_level = 0x08,
    min_position_limit = 0x09,
    max_position_limit = 0x0B,
    phase = 0x12,
    protection_condition = 0x13,
    angular_resolution = 0x1E,
    operating_mode = 0x21,
    protection_torque = 0x22,
    protection_time = 0x23,
    overload_torque = 0x24,
    torque_switch = 0x28,
    acceleration = 0x29,
    target_position = 0x2A,
    running_speed = 0x2E,
    torque_limit = 0x30,
    lock = 0x37,
    current_position = 0x38,
    servo_status = 0x41
  };

  STS3215() = default;
  ~STS3215() = default;
  STS3215(const STS3215 &) = delete;
  STS3215 &operator=(const STS3215 &) = delete;
  STS3215(STS3215 &&) = delete;
  STS3215 &operator=(STS3215 &&) = delete;

  [[nodiscard]] esp_err_t begin(STSCREATE &bus, uint8_t id);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] bool configurationValid() const { return configuration_valid_; }
  [[nodiscard]] esp_err_t refreshConfiguration();
  [[nodiscard]] uint8_t lastDeviceError() const { return last_device_error_; }
  [[nodiscard]] float degreesPerStep() const;
  [[nodiscard]] esp_err_t getOperatingMode(OperatingMode &mode) const;
  [[nodiscard]] esp_err_t verifyOperatingMode(OperatingMode expected) const;
  [[nodiscard]] esp_err_t setOperatingMode(OperatingMode mode,
                                           Persistence persistence);
  [[nodiscard]] esp_err_t configurePositionMode(Persistence persistence);
  [[nodiscard]] esp_err_t configureMultiTurnPositionMode(Persistence persistence);
  [[nodiscard]] esp_err_t configureStepMode(Persistence persistence);
  [[nodiscard]] esp_err_t setDirection(Direction direction,
                                       Persistence persistence);
  [[nodiscard]] esp_err_t setFeedbackMode(FeedbackMode mode,
                                          Persistence persistence);
  [[nodiscard]] esp_err_t setSpeedUnitMode(SpeedUnitMode mode,
                                           Persistence persistence);
  [[nodiscard]] esp_err_t enableTorque();
  [[nodiscard]] esp_err_t disableTorque();
  [[nodiscard]] esp_err_t setTorqueLimit(TorqueLimit limit);
  [[nodiscard]] esp_err_t readTorqueLimit(TorqueLimit &limit);
  [[nodiscard]] esp_err_t holdCurrentPosition(const HoldConfig &config);
  [[nodiscard]] esp_err_t moveAbsoluteDegrees(float degrees,
                                              const Motion &motion);
  [[nodiscard]] esp_err_t moveRelativeDegrees(float delta_degrees,
                                              const Motion &motion);
  [[nodiscard]] esp_err_t
  configureStallProtection(const StallProtection &config,
                           Persistence persistence);
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t readRaw(RawData &data);
  [[nodiscard]] esp_err_t read(Data &data);
  [[nodiscard]] esp_err_t readRegister(Register address, uint8_t *data,
                                       std::size_t length);
  [[nodiscard]] esp_err_t writeRegister(Register address, const uint8_t *data,
                                        std::size_t length,
                                        Persistence persistence);

private:
  struct ConfigurationSnapshot {
    uint8_t response_status_level{};
    uint8_t angular_resolution{};
    OperatingMode operating_mode{OperatingMode::position};
    uint8_t phase{};
    uint16_t minimum_position{};
    uint16_t maximum_position{};
  };

  [[nodiscard]] esp_err_t readConfiguration(STSCREATE &bus, uint8_t id,
                                            ConfigurationSnapshot &snapshot);
  void commitConfiguration(const ConfigurationSnapshot &snapshot);
  [[nodiscard]] esp_err_t readBytes(uint8_t address, uint8_t *data,
                                    std::size_t length);
  [[nodiscard]] esp_err_t writeBytes(uint8_t address, const uint8_t *data,
                                     std::size_t length);
  [[nodiscard]] esp_err_t writeEpRom(uint8_t address, const uint8_t *data,
                                     std::size_t length,
                                     Persistence persistence);
  [[nodiscard]] esp_err_t updatePhase(uint8_t mask, uint8_t value,
                                      Persistence persistence);
  [[nodiscard]] esp_err_t encodeMotion(float degrees, const Motion &motion,
                                       bool relative, uint8_t *data,
                                       std::size_t &length) const;
  [[nodiscard]] static Status decodeStatus(uint8_t raw);

  STSCREATE *bus_{nullptr};
  uint8_t id_{};
  OperatingMode operating_mode_{OperatingMode::position};
  uint8_t response_status_level_{};
  uint8_t angular_resolution_{1};
  uint8_t phase_{};
  uint16_t minimum_position_{};
  uint16_t maximum_position_{4095};
  uint8_t last_device_error_{};
  bool initialized_{false};
  bool configuration_valid_{false};
};
