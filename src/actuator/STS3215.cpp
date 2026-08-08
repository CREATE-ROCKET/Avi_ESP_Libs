#include "STS3215.h"

#include <array>
#include <cmath>
#include <limits>

namespace {
constexpr uint8_t kResponseStatusLevel = 0x08;
constexpr uint8_t kMinimumPosition = 0x09;
constexpr uint8_t kMaximumPosition = 0x0B;
constexpr uint8_t kPhase = 0x12;
constexpr uint8_t kProtectionCondition = 0x13;
constexpr uint8_t kAngularResolution = 0x1E;
constexpr uint8_t kOperatingMode = 0x21;
constexpr uint8_t kProtectionTorque = 0x22;
constexpr uint8_t kProtectionTime = 0x23;
constexpr uint8_t kOverloadTorque = 0x24;
constexpr uint8_t kTorqueSwitch = 0x28;
constexpr uint8_t kAcceleration = 0x29;
constexpr uint8_t kTorqueLimit = 0x30;
constexpr uint8_t kLock = 0x37;
constexpr uint8_t kCurrentPosition = 0x38;
constexpr std::size_t kTelemetryLength = 15;
constexpr uint8_t kDirectionBit = 0x80;
constexpr uint8_t kFeedbackBit = 0x10;
constexpr uint8_t kSpeedUnitBit = 0x04;
constexpr uint8_t kOverloadProtectionBit = 0x20;

constexpr uint16_t littleEndian(const uint8_t *data) {
  return static_cast<uint16_t>(uint16_t{data[0]} | (uint16_t{data[1]} << 8));
}
constexpr void putLittleEndian(uint16_t value, uint8_t *data) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
}
constexpr uint16_t encodeRelative(int32_t steps) {
  const uint16_t magnitude = static_cast<uint16_t>(steps < 0 ? -steps : steps);
  return static_cast<uint16_t>(magnitude | (steps < 0 ? 0x8000 : 0));
}
constexpr int16_t decodeSignedMagnitude15(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) != 0 ? static_cast<int16_t>(-magnitude) : magnitude;
}
constexpr int16_t decodeSignedMagnitude10(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & ~uint16_t{0x0400});
  return (raw & 0x0400) != 0 ? static_cast<int16_t>(-magnitude) : magnitude;
}
constexpr float degreesPerStep(uint8_t resolution) {
  return 360.0F / 4096.0F * resolution;
}
constexpr float torquePercent(uint16_t raw) { return raw / 10.0F; }
constexpr float absolute(float value) { return value < 0 ? -value : value; }
constexpr bool validTorquePercent(float value) {
  return value == value && value >= 0.0F && value <= 100.0F;
}
constexpr bool validTorqueRaw(uint16_t value) { return value <= 1000; }
constexpr uint16_t torqueRawFromPercent(float value) {
  return static_cast<uint16_t>(value * 10.0F + 0.5F);
}
constexpr uint16_t stallTimeRaw(uint16_t milliseconds) {
  return static_cast<uint16_t>((milliseconds + 5) / 10);
}
static_assert(absolute(degreesPerStep(1) - 0.087890625F) < 0.000001F);
static_assert(absolute(4096 * degreesPerStep(1) - 360.0F) < 0.001F);
static_assert(absolute(2048 * degreesPerStep(1) - 180.0F) < 0.001F);
static_assert(encodeRelative(100) == 0x0064);
static_assert(encodeRelative(-100) == 0x8064);
static_assert((encodeRelative(-100) & 0xFF) == 0x64);
static_assert((encodeRelative(-100) >> 8) == 0x80);
static_assert(torquePercent(1000) == 100.0F);
static_assert(torquePercent(500) == 50.0F);
static_assert(torquePercent(200) == 20.0F);
static_assert(1000 / 50 == 20);
static_assert(1000 / 100 == 10);
static_assert(absolute(100 * 0.0065F - 0.65F) < 0.00001F);
static_assert(absolute(74 * 0.1F - 7.4F) < 0.00001F);
static_assert(200 * 10 == 2000);
static_assert(decodeSignedMagnitude15(0x0000) == 0);
static_assert(decodeSignedMagnitude15(0x0064) == 100);
static_assert(decodeSignedMagnitude15(0x8064) == -100);
static_assert(decodeSignedMagnitude15(0x7FFF) == 32767);
static_assert(decodeSignedMagnitude10(0x0000) == 0);
static_assert(decodeSignedMagnitude10(0x0064) == 100);
static_assert(decodeSignedMagnitude10(0x0464) == -100);
static_assert(absolute(decodeSignedMagnitude15(0x0064) * 0.0065F - 0.65F) <
              0.00001F);
static_assert(absolute(decodeSignedMagnitude15(0x8064) * 0.0065F + 0.65F) <
              0.00001F);
static_assert(absolute(decodeSignedMagnitude15(0x0064) * degreesPerStep(1) -
                       8.7890625F) < 0.00001F);
static_assert(absolute(decodeSignedMagnitude15(0x8064) * degreesPerStep(1) +
                       8.7890625F) < 0.00001F);
static_assert(validTorquePercent(0.0F));
static_assert(validTorqueRaw(0));
static_assert(validTorqueRaw(1000));
static_assert(!validTorqueRaw(1001));
static_assert(torqueRawFromPercent(50.0F) == 500);
static_assert(torqueRawFromPercent(100.0F) == 1000);
static_assert(validTorquePercent(100.0F));
static_assert(!validTorquePercent(-1.0F));
static_assert(!validTorquePercent(101.0F));
static_assert(!validTorquePercent(std::numeric_limits<float>::quiet_NaN()));
static_assert(!validTorquePercent(std::numeric_limits<float>::infinity()));
static_assert(stallTimeRaw(0) == 0);
static_assert(stallTimeRaw(4) == 0);
static_assert(stallTimeRaw(5) == 1);
static_assert(stallTimeRaw(2540) == 254);

bool validMode(STS3215::OperatingMode mode) {
  return static_cast<uint8_t>(mode) <= 3;
}
bool validPersistence(STS3215::Persistence persistence) {
  return persistence == STS3215::Persistence::volatile_only ||
         persistence == STS3215::Persistence::persistent;
}
} // namespace

STS3215::TorqueLimit STS3215::TorqueLimit::raw(uint16_t value) {
  return TorqueLimit(validTorqueRaw(value) ? value : 0, validTorqueRaw(value));
}

STS3215::TorqueLimit STS3215::TorqueLimit::percent(float value) {
  if (!validTorquePercent(value))
    return TorqueLimit(0, false);
  return TorqueLimit(torqueRawFromPercent(value), true);
}

float STS3215::gearRatio() const {
  switch (model_) {
  case Model::c001_1_345:
    return 345.0F;
  case Model::c044_1_191:
    return 191.0F;
  case Model::c046_1_147:
    return 147.0F;
  }
  return 0.0F;
}

float STS3215::degreesPerStep() const {
  return ::degreesPerStep(angular_resolution_);
}

esp_err_t STS3215::readBytes(uint8_t address, uint8_t *data,
                             std::size_t length) {
  if (!initialized_ || bus_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t device_error{};
  const esp_err_t result =
      bus_->read(id_, address, data, length, &device_error);
  if (result == ESP_OK)
    last_device_error_ = device_error;
  return result;
}

esp_err_t STS3215::writeBytes(uint8_t address, const uint8_t *data,
                              std::size_t length) {
  if (!initialized_ || bus_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (response_status_level_ == 0)
    return bus_->write(id_, address, data, length, false);
  uint8_t device_error{};
  const esp_err_t result =
      bus_->write(id_, address, data, length, true, &device_error);
  if (result == ESP_OK)
    last_device_error_ = device_error;
  return result;
}

esp_err_t STS3215::begin(STSCREATE &bus, uint8_t id, Model model) {
  if (initialized_ || bus_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!bus.initialized() || id > 253 || static_cast<uint8_t>(model) > 2)
    return ESP_ERR_INVALID_ARG;
  uint8_t device_error{};
  esp_err_t result = bus.ping(id, &device_error);
  uint8_t response_level{};
  uint8_t resolution{};
  uint8_t mode{};
  uint8_t phase{};
  uint8_t minimum[2]{};
  uint8_t maximum[2]{};
  if (result == ESP_OK)
    result =
        bus.read(id, kResponseStatusLevel, &response_level, 1, &device_error);
  if (result == ESP_OK)
    result = bus.read(id, kAngularResolution, &resolution, 1, &device_error);
  if (result == ESP_OK)
    result = bus.read(id, kOperatingMode, &mode, 1, &device_error);
  if (result == ESP_OK)
    result = bus.read(id, kPhase, &phase, 1, &device_error);
  if (result == ESP_OK)
    result =
        bus.read(id, kMinimumPosition, minimum, sizeof(minimum), &device_error);
  if (result == ESP_OK)
    result =
        bus.read(id, kMaximumPosition, maximum, sizeof(maximum), &device_error);
  if (result != ESP_OK)
    return result;
  if (response_level > 1 || resolution == 0 || resolution > 3 || mode > 3)
    return ESP_ERR_INVALID_RESPONSE;
  bus_ = &bus;
  id_ = id;
  model_ = model;
  response_status_level_ = response_level;
  angular_resolution_ = resolution;
  operating_mode_ = static_cast<OperatingMode>(mode);
  phase_ = phase;
  minimum_position_ = littleEndian(minimum);
  maximum_position_ = littleEndian(maximum);
  last_device_error_ = device_error;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t STS3215::end() {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  initialized_ = false;
  bus_ = nullptr;
  return ESP_OK;
}

esp_err_t STS3215::getOperatingMode(OperatingMode &mode) const {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  mode = operating_mode_;
  return ESP_OK;
}

esp_err_t STS3215::verifyOperatingMode(OperatingMode expected) const {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  return operating_mode_ == expected ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t STS3215::writeEpRom(uint8_t address, const uint8_t *data,
                              std::size_t length, Persistence persistence) {
  if (!initialized_ || !validPersistence(persistence) || data == nullptr ||
      length == 0)
    return ESP_ERR_INVALID_ARG;
  uint8_t saved_lock{};
  esp_err_t operation = readBytes(kLock, &saved_lock, 1);
  const uint8_t desired = persistence == Persistence::persistent ? 0 : 1;
  bool lock_changed = false;
  if (operation == ESP_OK && saved_lock != desired) {
    operation = writeBytes(kLock, &desired, 1);
    lock_changed = operation == ESP_OK;
  }
  if (operation == ESP_OK)
    operation = writeBytes(address, data, length);
  // 対象registerの書込みに失敗してもlock flagは元へ戻す。
  // 対象register自体はrollbackせず、復元失敗を優先して返す。
  esp_err_t restore = ESP_OK;
  if (lock_changed)
    restore = writeBytes(kLock, &saved_lock, 1);
  return restore != ESP_OK ? restore : operation;
}

esp_err_t STS3215::setOperatingMode(OperatingMode mode,
                                    Persistence persistence) {
  if (!validMode(mode))
    return ESP_ERR_INVALID_ARG;
  const uint8_t raw = static_cast<uint8_t>(mode);
  const esp_err_t result = writeEpRom(kOperatingMode, &raw, 1, persistence);
  if (result == ESP_OK)
    operating_mode_ = mode;
  return result;
}

esp_err_t STS3215::configureStepMode(Persistence persistence) {
  const uint8_t values[]{0, 0, 0, 0};
  esp_err_t result =
      writeEpRom(kMinimumPosition, values, sizeof(values), persistence);
  const uint8_t mode = static_cast<uint8_t>(OperatingMode::step);
  if (result == ESP_OK)
    result = writeEpRom(kOperatingMode, &mode, 1, persistence);
  if (result == ESP_OK) {
    minimum_position_ = 0;
    maximum_position_ = 0;
    operating_mode_ = OperatingMode::step;
  }
  return result;
}

esp_err_t STS3215::updatePhase(uint8_t mask, uint8_t value,
                               Persistence persistence) {
  uint8_t next = static_cast<uint8_t>((phase_ & ~mask) | (value & mask));
  const esp_err_t result = writeEpRom(kPhase, &next, 1, persistence);
  if (result == ESP_OK)
    phase_ = next;
  return result;
}

esp_err_t STS3215::setDirection(Direction direction, Persistence persistence) {
  if (direction != Direction::normal && direction != Direction::reverse)
    return ESP_ERR_INVALID_ARG;
  return updatePhase(kDirectionBit,
                     direction == Direction::reverse ? kDirectionBit : 0,
                     persistence);
}

esp_err_t STS3215::setFeedbackMode(FeedbackMode mode, Persistence persistence) {
  if (mode != FeedbackMode::single_turn && mode != FeedbackMode::multi_turn)
    return ESP_ERR_INVALID_ARG;
  return updatePhase(kFeedbackBit,
                     mode == FeedbackMode::multi_turn ? kFeedbackBit : 0,
                     persistence);
}

esp_err_t STS3215::setSpeedUnitMode(SpeedUnitMode mode,
                                    Persistence persistence) {
  if (mode != SpeedUnitMode::steps50_per_second &&
      mode != SpeedUnitMode::step_per_second)
    return ESP_ERR_INVALID_ARG;
  return updatePhase(kSpeedUnitBit,
                     mode == SpeedUnitMode::step_per_second ? kSpeedUnitBit : 0,
                     persistence);
}

esp_err_t STS3215::enableTorque() {
  const uint8_t value = 1;
  return writeBytes(kTorqueSwitch, &value, 1);
}

esp_err_t STS3215::disableTorque() {
  const uint8_t value = 0;
  return writeBytes(kTorqueSwitch, &value, 1);
}

esp_err_t STS3215::setTorqueLimit(TorqueLimit limit) {
  if (!limit.valid())
    return ESP_ERR_INVALID_ARG;
  uint8_t raw[2]{};
  putLittleEndian(limit.rawValue(), raw);
  return writeBytes(kTorqueLimit, raw, sizeof(raw));
}

esp_err_t STS3215::readTorqueLimit(TorqueLimit &limit) {
  uint8_t raw[2]{};
  const esp_err_t result = readBytes(kTorqueLimit, raw, sizeof(raw));
  if (result != ESP_OK)
    return result;
  const uint16_t value = littleEndian(raw);
  if (value > 1000)
    return ESP_ERR_INVALID_RESPONSE;
  limit = TorqueLimit::raw(value);
  return ESP_OK;
}

esp_err_t STS3215::holdCurrentPosition(const HoldConfig &config) {
  if (!initialized_ || !config.torque_limit.valid())
    return ESP_ERR_INVALID_ARG;
  esp_err_t result = ESP_OK;
  if (operating_mode_ == OperatingMode::position) {
    uint8_t position[2]{};
    result = readBytes(kCurrentPosition, position, sizeof(position));
    if (result == ESP_OK)
      result = setTorqueLimit(config.torque_limit);
    if (result == ESP_OK)
      result = writeBytes(0x2A, position, sizeof(position));
  } else if (operating_mode_ == OperatingMode::step) {
    result = setTorqueLimit(config.torque_limit);
    const uint8_t zero[2]{};
    if (result == ESP_OK)
      result = writeBytes(0x2A, zero, sizeof(zero));
  } else {
    return ESP_ERR_INVALID_STATE;
  }
  return result == ESP_OK ? enableTorque() : result;
}

esp_err_t STS3215::encodeMotion(float degrees, const Motion &motion,
                                bool relative, uint8_t *data,
                                std::size_t &length) const {
  if (!std::isfinite(degrees) || !std::isfinite(motion.speed_deg_s) ||
      !std::isfinite(motion.acceleration_deg_s2) || motion.speed_deg_s <= 0 ||
      motion.acceleration_deg_s2 < 0 ||
      (motion.torque_limit && !motion.torque_limit->valid()))
    return ESP_ERR_INVALID_ARG;
  const double step_size = degreesPerStep();
  const long steps = std::lround(degrees / step_size);
  uint16_t target{};
  if (relative) {
    if (steps < -32766 || steps > 32766)
      return ESP_ERR_INVALID_ARG;
    target = encodeRelative(static_cast<int32_t>(steps));
  } else {
    if (steps < minimum_position_ || steps > maximum_position_ || steps > 65535)
      return ESP_ERR_INVALID_ARG;
    target = static_cast<uint16_t>(steps);
  }
  const double steps_per_second = motion.speed_deg_s / step_size;
  double speed_raw = (phase_ & kSpeedUnitBit) != 0 ? steps_per_second
                                                   : steps_per_second / 50.0;
  long speed = std::lround(speed_raw);
  if (speed < 1)
    speed = 1;
  if (speed > 0x7FFF)
    return ESP_ERR_INVALID_ARG;
  long acceleration =
      std::lround(motion.acceleration_deg_s2 / step_size / 100.0);
  if (motion.acceleration_deg_s2 > 0 && acceleration < 1)
    acceleration = 1;
  if (acceleration < 0 || acceleration > 254)
    return ESP_ERR_INVALID_ARG;
  data[0] = static_cast<uint8_t>(acceleration);
  putLittleEndian(target, &data[1]);
  data[3] = 0;
  data[4] = 0;
  putLittleEndian(static_cast<uint16_t>(speed), &data[5]);
  length = 7;
  if (motion.torque_limit) {
    putLittleEndian(motion.torque_limit->rawValue(), &data[7]);
    length = 9;
  }
  return ESP_OK;
}

esp_err_t STS3215::moveAbsoluteDegrees(float degrees, const Motion &motion) {
  if (operating_mode_ != OperatingMode::position)
    return ESP_ERR_INVALID_STATE;
  uint8_t data[9]{};
  std::size_t length{};
  const esp_err_t result = encodeMotion(degrees, motion, false, data, length);
  return result == ESP_OK ? writeBytes(kAcceleration, data, length) : result;
}

esp_err_t STS3215::moveRelativeDegrees(float degrees, const Motion &motion) {
  if (operating_mode_ != OperatingMode::step)
    return ESP_ERR_INVALID_STATE;
  if (degrees == 0.0F)
    return ESP_OK;
  uint8_t data[9]{};
  std::size_t length{};
  const esp_err_t result = encodeMotion(degrees, motion, true, data, length);
  return result == ESP_OK ? writeBytes(kAcceleration, data, length) : result;
}

esp_err_t STS3215::configureStallProtection(const StallProtection &config,
                                            Persistence persistence) {
  if (!std::isfinite(config.trigger_torque_percent) ||
      !std::isfinite(config.protected_torque_percent) ||
      config.trigger_torque_percent < 0 ||
      config.trigger_torque_percent > 100 ||
      config.protected_torque_percent < 0 ||
      config.protected_torque_percent > 100 || config.trigger_time_ms > 2540)
    return ESP_ERR_INVALID_ARG;
  const uint8_t trigger =
      static_cast<uint8_t>(std::lround(config.trigger_torque_percent));
  const uint8_t protected_torque =
      static_cast<uint8_t>(std::lround(config.protected_torque_percent));
  const uint16_t time_value = stallTimeRaw(config.trigger_time_ms);
  if (time_value > 254)
    return ESP_ERR_INVALID_ARG;
  const uint8_t time = static_cast<uint8_t>(time_value);
  esp_err_t result =
      writeEpRom(kProtectionTorque, &protected_torque, 1, persistence);
  if (result == ESP_OK)
    result = writeEpRom(kProtectionTime, &time, 1, persistence);
  if (result == ESP_OK)
    result = writeEpRom(kOverloadTorque, &trigger, 1, persistence);
  uint8_t flags{};
  if (result == ESP_OK)
    result = readBytes(kProtectionCondition, &flags, 1);
  if (result == ESP_OK) {
    flags = config.enabled
                ? static_cast<uint8_t>(flags | kOverloadProtectionBit)
                : static_cast<uint8_t>(flags & ~kOverloadProtectionBit);
    result = writeEpRom(kProtectionCondition, &flags, 1, persistence);
  }
  return result;
}

STS3215::Status STS3215::decodeStatus(uint8_t raw) {
  return {(raw & 0x20) != 0, (raw & 0x08) != 0, (raw & 0x04) != 0,
          (raw & 0x02) != 0, (raw & 0x01) != 0, raw};
}

esp_err_t STS3215::getStatus(Status &status) {
  uint8_t raw{};
  const esp_err_t result = readBytes(0x41, &raw, 1);
  if (result == ESP_OK)
    status = decodeStatus(raw);
  return result;
}

esp_err_t STS3215::readRaw(RawData &data) {
  uint8_t raw[kTelemetryLength]{};
  const esp_err_t result = readBytes(kCurrentPosition, raw, sizeof(raw));
  if (result != ESP_OK)
    return result;
  RawData next{};
  next.position = littleEndian(&raw[0]);
  next.speed = littleEndian(&raw[2]);
  next.load = littleEndian(&raw[4]);
  next.voltage = raw[6];
  next.temperature = raw[7];
  next.registered_instruction = raw[8] != 0;
  next.status = raw[9];
  next.moving = raw[10] != 0;
  next.current = littleEndian(&raw[13]);
  data = next;
  return ESP_OK;
}

esp_err_t STS3215::read(Data &data) {
  RawData raw{};
  const esp_err_t result = readRaw(raw);
  if (result != ESP_OK)
    return result;
  const float unit = (phase_ & kSpeedUnitBit) != 0 ? 1.0F : 50.0F;
  Data next{};
  next.position_deg = decodeSignedMagnitude15(raw.position) * degreesPerStep();
  next.speed_deg_s =
      decodeSignedMagnitude15(raw.speed) * unit * degreesPerStep();
  next.load_raw = decodeSignedMagnitude10(raw.load);
  next.voltage_v = raw.voltage * 0.1F;
  next.temperature_celsius = raw.temperature;
  next.current_a = decodeSignedMagnitude15(raw.current) * 0.0065F;
  next.moving = raw.moving;
  next.status = decodeStatus(raw.status);
  data = next;
  return ESP_OK;
}

esp_err_t STS3215::readRegister(Register address, uint8_t *data,
                                std::size_t length) {
  return readBytes(static_cast<uint8_t>(address), data, length);
}

esp_err_t STS3215::writeRegister(Register address, const uint8_t *data,
                                 std::size_t length, Persistence persistence) {
  const uint8_t raw_address = static_cast<uint8_t>(address);
  if (address == Register::id || address == Register::baud_rate)
    return ESP_ERR_NOT_SUPPORTED;
  if (raw_address < kTorqueSwitch)
    return writeEpRom(raw_address, data, length, persistence);
  return writeBytes(raw_address, data, length);
}
