#include "ICM20602.h"

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"

#include <cmath>

namespace {

constexpr uint8_t kSampleRateDivider = 0x19;
constexpr uint8_t kSelfTestGyro = 0x50;
constexpr uint8_t kSelfTestAccel = 0x0D;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kAccelConfig2 = 0x1D;
constexpr uint8_t kIntStatus = 0x3A;
constexpr uint8_t kAccelData = 0x3B;
constexpr uint8_t kUserControl = 0x6A;
constexpr uint8_t kPowerManagement = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kExpectedWhoAmI = 0x12;
constexpr uint32_t kMaximumSpiFrequency = 10000000;
constexpr std::size_t kSelfTestSamples = 200;

int16_t signedWord(const uint8_t *data) {
  return static_cast<int16_t>((uint16_t{data[0]} << 8) | data[1]);
}

bool accelBits(ICM20602::AccelRange range, uint8_t &bits) {
  switch (range) {
  case ICM20602::AccelRange::g2:
    bits = 0U << 3;
    return true;
  case ICM20602::AccelRange::g4:
    bits = 1U << 3;
    return true;
  case ICM20602::AccelRange::g8:
    bits = 2U << 3;
    return true;
  case ICM20602::AccelRange::g16:
    bits = 3U << 3;
    return true;
  }
  return false;
}

bool gyroBits(ICM20602::GyroRange range, uint8_t &bits) {
  switch (range) {
  case ICM20602::GyroRange::dps250:
    bits = 0U << 3;
    return true;
  case ICM20602::GyroRange::dps500:
    bits = 1U << 3;
    return true;
  case ICM20602::GyroRange::dps1000:
    bits = 2U << 3;
    return true;
  case ICM20602::GyroRange::dps2000:
    bits = 3U << 3;
    return true;
  }
  return false;
}

bool gyroDlpfBits(ICM20602::GyroDlpf dlpf, uint8_t &bits) {
  switch (dlpf) {
  case ICM20602::GyroDlpf::hz250:
    bits = 0;
    return true;
  case ICM20602::GyroDlpf::hz176:
    bits = 1;
    return true;
  case ICM20602::GyroDlpf::hz92:
    bits = 2;
    return true;
  case ICM20602::GyroDlpf::hz41:
    bits = 3;
    return true;
  case ICM20602::GyroDlpf::hz20:
    bits = 4;
    return true;
  case ICM20602::GyroDlpf::hz10:
    bits = 5;
    return true;
  case ICM20602::GyroDlpf::hz5:
    bits = 6;
    return true;
  case ICM20602::GyroDlpf::hz3281:
    bits = 7;
    return true;
  }
  return false;
}

bool accelDlpfBits(ICM20602::AccelDlpf dlpf, uint8_t &bits) {
  switch (dlpf) {
  case ICM20602::AccelDlpf::hz1046:
    bits = 0x08;
    return true;
  case ICM20602::AccelDlpf::hz218:
    bits = 0;
    return true;
  case ICM20602::AccelDlpf::hz99:
    bits = 2;
    return true;
  case ICM20602::AccelDlpf::hz44_8:
    bits = 3;
    return true;
  case ICM20602::AccelDlpf::hz21_2:
    bits = 4;
    return true;
  case ICM20602::AccelDlpf::hz10_2:
    bits = 5;
    return true;
  case ICM20602::AccelDlpf::hz5_1:
    bits = 6;
    return true;
  case ICM20602::AccelDlpf::hz420:
    bits = 7;
    return true;
  }
  return false;
}

float accelSensitivity(ICM20602::AccelRange range) {
  switch (range) {
  case ICM20602::AccelRange::g2:
    return 16384.0F;
  case ICM20602::AccelRange::g4:
    return 8192.0F;
  case ICM20602::AccelRange::g8:
    return 4096.0F;
  case ICM20602::AccelRange::g16:
    return 2048.0F;
  }
  return 1.0F;
}

float gyroSensitivity(ICM20602::GyroRange range) {
  switch (range) {
  case ICM20602::GyroRange::dps250:
    return 131.0F;
  case ICM20602::GyroRange::dps500:
    return 65.5F;
  case ICM20602::GyroRange::dps1000:
    return 32.8F;
  case ICM20602::GyroRange::dps2000:
    return 16.4F;
  }
  return 1.0F;
}

float factoryTrim(uint8_t code) {
  return code == 0 ? 0.0F : 2620.0F * std::pow(1.01F, code - 1);
}

bool factoryCodesValid(const uint8_t (&codes)[3]) {
  return codes[0] != 0 && codes[1] != 0 && codes[2] != 0;
}

constexpr bool accelOtpPass(float measured, float trim) {
  return measured > trim * 0.5F && measured < trim * 1.5F;
}

constexpr bool gyroOtpPass(float measured, float trim) {
  return measured > trim * 0.5F;
}

constexpr bool accelFallbackPass(float measured) {
  return measured >= 225.0F * 16384.0F / 1000.0F &&
         measured <= 675.0F * 16384.0F / 1000.0F;
}

constexpr bool gyroFallbackPass(float measured) {
  return measured >= 60.0F * 131.0F;
}

constexpr bool gyroOffsetPass(float baseline) {
  return baseline <= 20.0F * 131.0F;
}

bool accelSelfTestAxis(int32_t response, uint8_t code, bool otp_valid) {
  const float measured = std::fabs(static_cast<float>(response));
  if (!otp_valid)
    return accelFallbackPass(measured);
  const float trim = factoryTrim(code);
  return accelOtpPass(measured, trim);
}

bool gyroSelfTestAxis(int32_t response, int32_t baseline, uint8_t code,
                      bool otp_valid) {
  const float measured = std::fabs(static_cast<float>(response));
  const bool response_ok = otp_valid ? gyroOtpPass(measured, factoryTrim(code))
                                     : gyroFallbackPass(measured);
  const bool offset_ok =
      gyroOffsetPass(std::fabs(static_cast<float>(baseline)));
  return response_ok && offset_ok;
}

static_assert(!accelOtpPass(499.0F, 1000.0F));
static_assert(accelOtpPass(1000.0F, 1000.0F));
static_assert(!accelOtpPass(1501.0F, 1000.0F));
static_assert(gyroOtpPass(2000.0F, 1000.0F));
static_assert(!gyroOtpPass(500.0F, 1000.0F));
static_assert(accelFallbackPass(225.0F * 16384.0F / 1000.0F));
static_assert(accelFallbackPass(675.0F * 16384.0F / 1000.0F));
static_assert(gyroFallbackPass(60.0F * 131.0F));
static_assert(gyroOffsetPass(20.0F * 131.0F));
static_assert(!gyroOffsetPass(20.0F * 131.0F + 1.0F));

} // namespace

ICM20602::~ICM20602() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t ICM20602::begin(SPICREATE &spi, int chip_select,
                          uint32_t frequency_hz) {
  Config config{};
  config.frequency_hz = frequency_hz;
  return begin(spi, chip_select, config);
}

esp_err_t ICM20602::begin(SPICREATE &spi, int chip_select,
                          const Config &config) {
  if (device_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t accel{};
  uint8_t gyro{};
  uint8_t accel_dlpf{};
  uint8_t gyro_dlpf{};
  if (config.frequency_hz == 0 || config.frequency_hz > kMaximumSpiFrequency ||
      !accelBits(config.accel_range, accel) ||
      !gyroBits(config.gyro_range, gyro) ||
      !accelDlpfBits(config.accel_dlpf, accel_dlpf) ||
      !gyroDlpfBits(config.gyro_dlpf, gyro_dlpf))
    return ESP_ERR_INVALID_ARG;

  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 0, 1}, device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;
  result = spi_->writeRegister(device_, kPowerManagement, 0x80);
  if (result == ESP_OK)
    avi_delay_ms(100);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kPowerManagement, 0x01);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kUserControl, 0x10);
  uint8_t identity{};
  if (result == ESP_OK)
    result = spi_->readRegister(device_, kWhoAmI | 0x80, identity);
  if (result == ESP_OK && identity != kExpectedWhoAmI)
    result = ESP_ERR_INVALID_RESPONSE;
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kConfig, gyro_dlpf);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kSampleRateDivider,
                                 config.sample_rate_divider);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kGyroConfig, gyro);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kAccelConfig, accel);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kAccelConfig2, accel_dlpf);
  if (result != ESP_OK) {
    const esp_err_t cleanup_result = end();
    return cleanup_result == ESP_OK ? result : cleanup_result;
  }
  accel_range_ = config.accel_range;
  gyro_range_ = config.gyro_range;
  config_ = config;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t ICM20602::end() {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  initialized_ = false;
  const esp_err_t sleep_result =
      spi_->writeRegister(device_, kPowerManagement, 0x40);
  const esp_err_t remove_result = spi_->removeDevice(device_);
  if (remove_result == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
    config_ = Config{};
  }
  return remove_result != ESP_OK ? remove_result : sleep_result;
}

esp_err_t ICM20602::whoAmI(uint8_t &value) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, kWhoAmI | 0x80, value);
}

esp_err_t ICM20602::getStatus(Status &status) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw{};
  const esp_err_t result = spi_->readRegister(device_, kIntStatus | 0x80, raw);
  if (result == ESP_OK) {
    Status next{};
    next.data_ready = (raw & 0x01) != 0;
    status = next;
  }
  return result;
}

esp_err_t ICM20602::available(bool &ready) {
  Status status{};
  const esp_err_t result = getStatus(status);
  if (result == ESP_OK)
    ready = status.data_ready;
  return result;
}

bool ICM20602::available() {
  bool ready{};
  return available(ready) == ESP_OK && ready;
}

esp_err_t ICM20602::readRaw(RawData &data) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t bytes[14]{};
  const esp_err_t result =
      spi_->read(device_, kAccelData | 0x80, bytes, sizeof(bytes));
  if (result != ESP_OK)
    return result;
  RawData next{};
  for (std::size_t i = 0; i < next.acceleration.size(); ++i)
    next.acceleration[i] = signedWord(&bytes[i * 2]);
  next.temperature = signedWord(&bytes[6]);
  for (std::size_t i = 0; i < next.angular_velocity.size(); ++i)
    next.angular_velocity[i] = signedWord(&bytes[8 + i * 2]);
  data = next;
  return ESP_OK;
}

esp_err_t ICM20602::read(Data &data) {
  RawData raw{};
  const esp_err_t result = readRaw(raw);
  if (result != ESP_OK)
    return result;
  Data next{};
  const float accel_scale = accelSensitivity(accel_range_);
  const float gyro_scale = gyroSensitivity(gyro_range_);
  for (std::size_t i = 0; i < raw.acceleration.size(); ++i) {
    next.acceleration_g[i] = raw.acceleration[i] / accel_scale;
    next.angular_velocity_dps[i] = raw.angular_velocity[i] / gyro_scale;
  }
  next.temperature_celsius = raw.temperature / 326.8F + 25.0F;
  data = next;
  return ESP_OK;
}

esp_err_t ICM20602::selfTest(SelfTestResult &result, avi::Timeout timeout) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint64_t timeout_ms{};
  if (!timeout.isFinite() || !timeout.millisecondsValue(timeout_ms) ||
      timeout_ms == 0)
    return ESP_ERR_INVALID_ARG;

  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(timeout, deadline) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  const Config saved = config_;
  SelfTestResult next{};
  uint8_t gyro_codes[3]{};
  uint8_t accel_codes[3]{};
  esp_err_t operation = ESP_OK;
  for (std::size_t i = 0; i < 3 && operation == ESP_OK; ++i)
    operation = spi_->readRegister(
        device_, static_cast<uint8_t>(kSelfTestGyro + i) | 0x80, gyro_codes[i]);
  for (std::size_t i = 0; i < 3 && operation == ESP_OK; ++i)
    operation = spi_->readRegister(
        device_, static_cast<uint8_t>(kSelfTestAccel + i) | 0x80,
        accel_codes[i]);

  const auto writeTestConfig = [this](bool accel_stimulated,
                                      bool gyro_stimulated) {
    esp_err_t error = spi_->writeRegister(device_, kConfig, 2);
    if (error == ESP_OK)
      error = spi_->writeRegister(device_, kSampleRateDivider, 0);
    if (error == ESP_OK)
      error = spi_->writeRegister(device_, kGyroConfig,
                                  gyro_stimulated ? 0xE0 : 0x00);
    if (error == ESP_OK)
      error = spi_->writeRegister(device_, kAccelConfig,
                                  accel_stimulated ? 0xE0 : 0x00);
    if (error == ESP_OK)
      error = spi_->writeRegister(device_, kAccelConfig2, 2);
    return error;
  };
  const auto collect = [this, &deadline](std::array<int32_t, 3> &accel,
                                         std::array<int32_t, 3> &gyro) {
    std::array<int64_t, 3> accel_sum{};
    std::array<int64_t, 3> gyro_sum{};
    for (std::size_t sample = 0; sample < kSelfTestSamples; ++sample) {
      if (avi::internal::expired(deadline))
        return ESP_ERR_TIMEOUT;
      RawData raw{};
      const esp_err_t error = readRaw(raw);
      if (error != ESP_OK)
        return error;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        accel_sum[axis] += raw.acceleration[axis];
        gyro_sum[axis] += raw.angular_velocity[axis];
      }
      avi_delay_ms(1);
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      accel[axis] = static_cast<int32_t>(accel_sum[axis] / kSelfTestSamples);
      gyro[axis] = static_cast<int32_t>(gyro_sum[axis] / kSelfTestSamples);
    }
    return ESP_OK;
  };

  if (operation == ESP_OK)
    operation = writeTestConfig(false, false);
  if (operation == ESP_OK) {
    avi_delay_ms(20);
    operation = collect(next.accel_baseline, next.gyro_baseline);
  }
  if (operation == ESP_OK)
    operation = writeTestConfig(true, false);
  if (operation == ESP_OK) {
    avi_delay_ms(20);
    std::array<int32_t, 3> ignored_gyro{};
    operation = collect(next.accel_stimulated, ignored_gyro);
  }
  if (operation == ESP_OK)
    operation = writeTestConfig(false, true);
  if (operation == ESP_OK) {
    avi_delay_ms(20);
    std::array<int32_t, 3> ignored_accel{};
    operation = collect(ignored_accel, next.gyro_stimulated);
  }
  if (operation == ESP_OK) {
    const bool accel_otp_valid = factoryCodesValid(accel_codes);
    const bool gyro_otp_valid = factoryCodesValid(gyro_codes);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      next.accel_response[axis] =
          next.accel_stimulated[axis] - next.accel_baseline[axis];
      next.gyro_response[axis] =
          next.gyro_stimulated[axis] - next.gyro_baseline[axis];
      next.accel_passed[axis] = accelSelfTestAxis(
          next.accel_response[axis], accel_codes[axis], accel_otp_valid);
      next.gyro_passed[axis] =
          gyroSelfTestAxis(next.gyro_response[axis], next.gyro_baseline[axis],
                           gyro_codes[axis], gyro_otp_valid);
    }
    next.passed = next.accel_passed[0] && next.accel_passed[1] &&
                  next.accel_passed[2] && next.gyro_passed[0] &&
                  next.gyro_passed[1] && next.gyro_passed[2];
  }

  uint8_t accel{};
  uint8_t gyro{};
  uint8_t accel_dlpf{};
  uint8_t gyro_dlpf{};
  esp_err_t restore = accelBits(saved.accel_range, accel) &&
                              gyroBits(saved.gyro_range, gyro) &&
                              accelDlpfBits(saved.accel_dlpf, accel_dlpf) &&
                              gyroDlpfBits(saved.gyro_dlpf, gyro_dlpf)
                          ? ESP_OK
                          : ESP_ERR_INVALID_ARG;
  const auto restoreRegister = [this, &restore](uint8_t address,
                                                uint8_t value) {
    const esp_err_t error = spi_->writeRegister(device_, address, value);
    if (restore == ESP_OK && error != ESP_OK)
      restore = error;
  };
  restoreRegister(kGyroConfig, gyro);
  restoreRegister(kAccelConfig, accel);
  restoreRegister(kConfig, gyro_dlpf);
  restoreRegister(kAccelConfig2, accel_dlpf);
  restoreRegister(kSampleRateDivider, saved.sample_rate_divider);
  next.restored = restore == ESP_OK;
  if (restore == ESP_OK) {
    config_ = saved;
    accel_range_ = saved.accel_range;
    gyro_range_ = saved.gyro_range;
  }
  result = next;
  return restore != ESP_OK ? restore : operation;
}
