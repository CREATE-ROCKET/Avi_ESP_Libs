#include "LPS25HB.h"

#include "avi_esp_libs/compatibility.h"

namespace {
constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kExpectedWhoAmI = 0xBD;
constexpr uint8_t kResolution = 0x10;
constexpr uint8_t kControl1 = 0x20;
constexpr uint8_t kControl2 = 0x21;
constexpr uint8_t kStatus = 0x27;
constexpr uint8_t kPressureOutput = 0x28;

constexpr uint8_t kRead = 0x80;
constexpr uint8_t kAutoIncrement = 0x40;
constexpr uint8_t kPowerDown = 0x00;
constexpr uint8_t kPowerOn = 0x80;
constexpr uint8_t kBlockDataUpdate = 0x04;
constexpr uint8_t kI2cDisable = 0x08;
constexpr uint8_t kSoftwareReset = 0x04;
constexpr uint8_t kOneShot = 0x01;
constexpr uint8_t kPressureReady = 0x02;
constexpr uint8_t kTemperatureReady = 0x01;
constexpr uint8_t kPressureOverrun = 0x20;
constexpr uint8_t kTemperatureOverrun = 0x10;

constexpr uint32_t kMaximumSpiFrequencyHz = 10000000;
constexpr int64_t kResetTimeoutUs = 100000;

bool validConfig(const LPS25HB::Config &config) {
  const auto odr = static_cast<uint8_t>(config.odr);
  const auto pressure = static_cast<uint8_t>(config.pressure_average);
  const auto temperature = static_cast<uint8_t>(config.temperature_average);
  return config.frequency_hz > 0 &&
         config.frequency_hz <= kMaximumSpiFrequencyHz && odr <= 4 &&
         pressure <= 3 && temperature <= 3 &&
         (config.odr != LPS25HB::Odr::one_shot ||
          config.one_shot_timeout_ms > 0);
}
} // 名前なし名前空間

LPS25HB::~LPS25HB() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t LPS25HB::begin(SPICREATE &spi, int chip_select,
                         uint32_t frequency_hz) {
  Config config{};
  config.frequency_hz = frequency_hz;
  return begin(spi, chip_select, config);
}

esp_err_t LPS25HB::begin(SPICREATE &spi, int chip_select,
                         const Config &config) {
  if (spi_ != nullptr || device_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!validConfig(config))
    return ESP_ERR_INVALID_ARG;

  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 3, 1}, device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;

  const auto fail = [this](esp_err_t cause) {
    // 初期化途中で登録したデバイスハンドルを確実に解放する。
    const esp_err_t cleanup = spi_->removeDevice(device_);
    if (cleanup == ESP_OK)
      spi_ = nullptr;
    return cleanup == ESP_OK ? cause : cleanup;
  };

  uint8_t identity = 0;
  result = spi_->readRegister(device_, kRead | kWhoAmI, identity);
  if (result != ESP_OK)
    return fail(result);
  if (identity != kExpectedWhoAmI)
    return fail(ESP_ERR_INVALID_RESPONSE);

  result = spi_->writeRegister(device_, kControl2,
                               kI2cDisable | kSoftwareReset);
  if (result != ESP_OK)
    return fail(result);

  avi_delay_ms(1);
  const int64_t reset_deadline = avi_micros() + kResetTimeoutUs;
  while (true) {
    uint8_t control = 0;
    result = spi_->readRegister(device_, kRead | kControl2, control);
    if (result != ESP_OK)
      return fail(result);
    if ((control & kSoftwareReset) == 0)
      break;
    if (avi_micros() >= reset_deadline)
      return fail(ESP_ERR_TIMEOUT);
    avi_delay_ms(1);
  }

  result = spi_->writeRegister(device_, kControl2, kI2cDisable);
  if (result != ESP_OK)
    return fail(result);

  const uint8_t resolution = static_cast<uint8_t>(
      (static_cast<uint8_t>(config.temperature_average) << 2) |
      static_cast<uint8_t>(config.pressure_average));
  result = spi_->writeRegister(device_, kResolution, resolution);
  if (result != ESP_OK)
    return fail(result);

  uint8_t control1 = kBlockDataUpdate;
  if (config.odr != Odr::one_shot) {
    control1 |= kPowerOn |
                static_cast<uint8_t>(static_cast<uint8_t>(config.odr) << 4);
  }
  result = spi_->writeRegister(device_, kControl1, control1);
  if (result != ESP_OK)
    return fail(result);

  config_ = config;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t LPS25HB::end() {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;

  initialized_ = false;
  const esp_err_t power_down =
      spi_->writeRegister(device_, kControl1, kPowerDown);
  const esp_err_t remove = spi_->removeDevice(device_);
  if (remove != ESP_OK)
    return remove;

  spi_ = nullptr;
  config_ = Config{};
  return power_down;
}

esp_err_t LPS25HB::whoAmI(uint8_t &value) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, kRead | kWhoAmI, value);
}

esp_err_t LPS25HB::getStatus(Status &status) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;

  uint8_t raw = 0;
  const esp_err_t result =
      spi_->readRegister(device_, kRead | kStatus, raw);
  if (result != ESP_OK)
    return result;

  Status next{};
  next.pressure_ready = (raw & kPressureReady) != 0;
  next.temperature_ready = (raw & kTemperatureReady) != 0;
  next.pressure_overrun = (raw & kPressureOverrun) != 0;
  next.temperature_overrun = (raw & kTemperatureOverrun) != 0;
  status = next;
  return ESP_OK;
}

esp_err_t LPS25HB::get(Data &data) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;

  if (config_.odr == Odr::one_shot) {
    esp_err_t result =
        spi_->writeRegister(device_, kControl2, kI2cDisable | kOneShot);
    if (result != ESP_OK)
      return result;

    const int64_t deadline =
        avi_micros() + int64_t{config_.one_shot_timeout_ms} * 1000;
    while (true) {
      uint8_t control2 = 0;
      result = spi_->readRegister(device_, kRead | kControl2, control2);
      if (result != ESP_OK)
        return result;

      Status status{};
      result = getStatus(status);
      if (result != ESP_OK)
        return result;
      if ((control2 & kOneShot) == 0 && status.pressure_ready &&
          status.temperature_ready)
        break;
      if (avi_micros() >= deadline)
        return ESP_ERR_TIMEOUT;
      avi_delay_ms(1);
    }
  } else {
    Status status{};
    const esp_err_t result = getStatus(status);
    if (result != ESP_OK)
      return result;
    if (!status.pressure_ready || !status.temperature_ready)
      return ESP_ERR_NOT_FINISHED;
  }

  uint8_t raw[5]{};
  const esp_err_t result =
      spi_->read(device_, kRead | kAutoIncrement | kPressureOutput, raw,
                 sizeof(raw));
  if (result != ESP_OK)
    return result;

  const uint32_t pressure_bits = uint32_t{raw[0]} |
                                 (uint32_t{raw[1]} << 8) |
                                 (uint32_t{raw[2]} << 16);
  int32_t pressure_raw = static_cast<int32_t>(pressure_bits);
  if ((pressure_bits & 0x00800000U) != 0)
    pressure_raw -= 0x01000000;

  int32_t temperature_value =
      static_cast<int32_t>(uint32_t{raw[3]} | (uint32_t{raw[4]} << 8));
  if ((temperature_value & 0x00008000) != 0)
    temperature_value -= 0x00010000;
  const int16_t temperature_raw = static_cast<int16_t>(temperature_value);

  Data next{};
  next.pressure_raw = pressure_raw;
  next.pressure_pa = static_cast<float>(pressure_raw) * 100.0F / 4096.0F;
  next.temperature_raw = temperature_raw;
  next.temperature_celsius =
      42.5F + static_cast<float>(temperature_raw) / 480.0F;
  data = next;
  return ESP_OK;
}
