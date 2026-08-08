#include "SSCDRRN005PD2A5.h"

#include <array>

namespace {
constexpr uint8_t kAddress = 0x28;
constexpr uint16_t kPressureMinimumCount = 1638;
constexpr uint16_t kPressureMaximumCount = 14746;
constexpr float kMinimumPsi = -5.0F;
constexpr float kMaximumPsi = 5.0F;
constexpr float kPascalPerPsi = 6894.757293168F;

constexpr SSCDRRN005PD2A5::RawData decode(const std::array<uint8_t, 4> &raw) {
  return {static_cast<uint16_t>(((raw[0] & 0x3FU) << 8) | raw[1]),
          static_cast<uint16_t>((uint16_t{raw[2]} << 3) | (raw[3] >> 5)),
          static_cast<SSCDRRN005PD2A5::SensorStatus>(raw[0] >> 6)};
}

constexpr float pressurePa(uint16_t count) {
  return (kMinimumPsi + (static_cast<float>(count) - kPressureMinimumCount) *
                            (kMaximumPsi - kMinimumPsi) /
                            (kPressureMaximumCount - kPressureMinimumCount)) *
         kPascalPerPsi;
}

constexpr float temperatureCelsius(uint16_t count) {
  return static_cast<float>(count) * 200.0F / 2047.0F - 50.0F;
}

constexpr float absolute(float value) { return value < 0 ? -value : value; }
static_assert(absolute(pressurePa(1638) + 34473.786F) < 0.01F);
static_assert(absolute(pressurePa(14746) - 34473.786F) < 0.01F);
static_assert(temperatureCelsius(0) == -50.0F);
static_assert(temperatureCelsius(2047) == 150.0F);
static_assert(decode({0x00, 0x00, 0x00, 0x00}).status ==
              SSCDRRN005PD2A5::SensorStatus::normal);
static_assert(decode({0x40, 0x00, 0x00, 0x00}).status ==
              SSCDRRN005PD2A5::SensorStatus::command_mode);
static_assert(decode({0x80, 0x00, 0x00, 0x00}).status ==
              SSCDRRN005PD2A5::SensorStatus::stale);
static_assert(decode({0xC0, 0x00, 0x00, 0x00}).status ==
              SSCDRRN005PD2A5::SensorStatus::diagnostic_fault);
} // namespace

SSCDRRN005PD2A5::~SSCDRRN005PD2A5() {
  if (device_ != I2CCREATE::kInvalidDevice)
    (void)end();
}

esp_err_t SSCDRRN005PD2A5::begin(I2CCREATE &i2c) {
  if (i2c_ != nullptr || device_ != I2CCREATE::kInvalidDevice)
    return ESP_ERR_INVALID_STATE;
  if (!i2c.initialized() || i2c.frequencyHz() > 400000)
    return ESP_ERR_INVALID_ARG;
  esp_err_t result = i2c.probe(kAddress);
  if (result != ESP_OK)
    return result;
  result = i2c.addDevice({kAddress}, device_);
  if (result != ESP_OK)
    return result;
  i2c_ = &i2c;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t SSCDRRN005PD2A5::end() {
  if (i2c_ == nullptr || device_ == I2CCREATE::kInvalidDevice)
    return ESP_ERR_INVALID_STATE;
  initialized_ = false;
  const esp_err_t result = i2c_->removeDevice(device_);
  if (result == ESP_OK)
    i2c_ = nullptr;
  return result;
}

esp_err_t SSCDRRN005PD2A5::readRaw(RawData &data) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  std::array<uint8_t, 4> bytes{};
  const esp_err_t result = i2c_->read(device_, bytes.data(), bytes.size());
  if (result != ESP_OK)
    return result;
  data = decode(bytes);
  return ESP_OK;
}

esp_err_t SSCDRRN005PD2A5::read(Data &data) {
  RawData raw{};
  const esp_err_t result = readRaw(raw);
  if (result != ESP_OK)
    return result;
  switch (raw.status) {
  case SensorStatus::normal:
    break;
  case SensorStatus::stale:
    return ESP_ERR_NOT_FINISHED;
  case SensorStatus::command_mode:
    return ESP_ERR_INVALID_STATE;
  case SensorStatus::diagnostic_fault:
    return ESP_ERR_INVALID_RESPONSE;
  }
  Data next{};
  next.differential_pressure_pa = pressurePa(raw.pressure_counts);
  next.temperature_celsius = temperatureCelsius(raw.temperature_counts);
  data = next;
  return ESP_OK;
}
