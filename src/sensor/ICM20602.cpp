#include "ICM20602.h"

#include "avi_esp_libs/compatibility.h"

namespace {

constexpr uint8_t kSampleRateDivider = 0x19;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kIntStatus = 0x3A;
constexpr uint8_t kAccelData = 0x3B;
constexpr uint8_t kUserControl = 0x6A;
constexpr uint8_t kPowerManagement = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kExpectedWhoAmI = 0x12;
constexpr uint32_t kMaximumSpiFrequency = 10000000;

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

bool dlpfBits(ICM20602::Dlpf dlpf, uint8_t &bits) {
  switch (dlpf) {
  case ICM20602::Dlpf::hz250:
    bits = 0;
    return true;
  case ICM20602::Dlpf::hz176:
    bits = 1;
    return true;
  case ICM20602::Dlpf::hz92:
    bits = 2;
    return true;
  case ICM20602::Dlpf::hz41:
    bits = 3;
    return true;
  case ICM20602::Dlpf::hz20:
    bits = 4;
    return true;
  case ICM20602::Dlpf::hz10:
    bits = 5;
    return true;
  case ICM20602::Dlpf::hz5:
    bits = 6;
    return true;
  case ICM20602::Dlpf::hz3281:
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

} // 名前なし名前空間

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
  uint8_t dlpf{};
  if (config.frequency_hz == 0 || config.frequency_hz > kMaximumSpiFrequency ||
      !accelBits(config.accel_range, accel) ||
      !gyroBits(config.gyro_range, gyro) || !dlpfBits(config.dlpf, dlpf))
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
    result = spi_->writeRegister(device_, kConfig, dlpf);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kSampleRateDivider,
                                 config.sample_rate_divider);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kGyroConfig, gyro);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kAccelConfig, accel);
  if (result != ESP_OK) {
    const esp_err_t cleanup_result = end();
    return cleanup_result == ESP_OK ? result : cleanup_result;
  }
  accel_range_ = config.accel_range;
  gyro_range_ = config.gyro_range;
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
  const esp_err_t result =
      spi_->readRegister(device_, kIntStatus | 0x80, raw);
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
