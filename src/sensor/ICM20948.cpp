#include "ICM20948.h"

#include "avi_esp_libs/compatibility.h"

namespace {

constexpr uint8_t kRead = 0x80;
constexpr uint8_t kBank0 = 0x00;
constexpr uint8_t kBank2 = 0x20;
constexpr uint8_t kBank3 = 0x30;

constexpr uint8_t kWhoAmI = 0x00;
constexpr uint8_t kUserControl = 0x03;
constexpr uint8_t kPowerManagement1 = 0x06;
constexpr uint8_t kPowerManagement2 = 0x07;
constexpr uint8_t kInterruptPinConfig = 0x0F;
constexpr uint8_t kI2cMasterStatus = 0x17;
constexpr uint8_t kAccelOutput = 0x2D;
constexpr uint8_t kExternalSensorData = 0x3B;
constexpr uint8_t kDataReadyStatus = 0x74;
constexpr uint8_t kRegisterBankSelect = 0x7F;

constexpr uint8_t kGyroSampleRateDivider = 0x00;
constexpr uint8_t kGyroConfig = 0x01;
constexpr uint8_t kAccelSampleRateDividerHigh = 0x10;
constexpr uint8_t kAccelSampleRateDividerLow = 0x11;
constexpr uint8_t kAccelConfig = 0x14;

constexpr uint8_t kI2cMasterControl = 0x01;
constexpr uint8_t kI2cSlave0Address = 0x03;
constexpr uint8_t kI2cSlave0Register = 0x04;
constexpr uint8_t kI2cSlave0Control = 0x05;
constexpr uint8_t kI2cSlave4Address = 0x13;
constexpr uint8_t kI2cSlave4Register = 0x14;
constexpr uint8_t kI2cSlave4Control = 0x15;
constexpr uint8_t kI2cSlave4Output = 0x16;
constexpr uint8_t kI2cSlave4Input = 0x17;

constexpr uint8_t kExpectedWhoAmI = 0xEA;
constexpr uint8_t kDeviceReset = 0x80;
constexpr uint8_t kSleep = 0x40;
constexpr uint8_t kAutomaticClock = 0x01;
constexpr uint8_t kI2cMasterEnable = 0x20;
constexpr uint8_t kI2cInterfaceDisable = 0x10;
constexpr uint8_t kBypassEnable = 0x02;
constexpr uint8_t kI2cSlaveEnable = 0x80;
constexpr uint8_t kI2cSlave4Done = 0x40;
constexpr uint8_t kAuxiliaryI2cErrors = 0x3F;
constexpr uint8_t kI2cSlave4Nack = 0x10;
constexpr uint8_t kI2cLostArbitration = 0x20;

constexpr uint8_t kMagnetometerAddress = 0x0C;
constexpr uint8_t kMagnetometerWhoAmI1 = 0x00;
constexpr uint8_t kMagnetometerWhoAmI2 = 0x01;
constexpr uint8_t kMagnetometerStatus1 = 0x10;
constexpr uint8_t kMagnetometerControl2 = 0x31;
constexpr uint8_t kMagnetometerControl3 = 0x32;
constexpr uint8_t kExpectedMagnetometerWhoAmI1 = 0x48;
constexpr uint8_t kExpectedMagnetometerWhoAmI2 = 0x09;
constexpr uint8_t kMagnetometerDataReady = 0x01;
constexpr uint8_t kMagnetometerOverrun = 0x02;
constexpr uint8_t kMagnetometerOverflow = 0x08;

constexpr uint32_t kMaximumSpiFrequencyHz = 7000000;
constexpr uint16_t kMaximumAccelSampleRateDivider = 4095;
constexpr uint32_t kGyroscopeStartupDelayMs = 35;

bool validDlpf(ICM20948::Dlpf dlpf) {
  switch (dlpf) {
  case ICM20948::Dlpf::level0:
  case ICM20948::Dlpf::level1:
  case ICM20948::Dlpf::level2:
  case ICM20948::Dlpf::level3:
  case ICM20948::Dlpf::level4:
  case ICM20948::Dlpf::level5:
  case ICM20948::Dlpf::level6:
  case ICM20948::Dlpf::level7:
  case ICM20948::Dlpf::bypass:
    return true;
  }
  return false;
}

bool validMagnetometerOdr(ICM20948::MagnetometerOdr odr) {
  switch (odr) {
  case ICM20948::MagnetometerOdr::off:
  case ICM20948::MagnetometerOdr::hz10:
  case ICM20948::MagnetometerOdr::hz20:
  case ICM20948::MagnetometerOdr::hz50:
  case ICM20948::MagnetometerOdr::hz100:
    return true;
  }
  return false;
}

bool validConfig(const ICM20948::Config &config) {
  return config.frequency_hz > 0 &&
         config.frequency_hz <= kMaximumSpiFrequencyHz &&
         static_cast<uint8_t>(config.accel_range) <= 3 &&
         static_cast<uint8_t>(config.gyro_range) <= 3 &&
         config.accel_sample_rate_divider <= kMaximumAccelSampleRateDivider &&
         validDlpf(config.accel_dlpf) && validDlpf(config.gyro_dlpf) &&
         validMagnetometerOdr(config.magnetometer_odr) &&
         config.operation_timeout_ms > 0;
}

uint8_t sensorConfig(uint8_t range, ICM20948::Dlpf dlpf) {
  uint8_t value = static_cast<uint8_t>(range << 1);
  if (dlpf != ICM20948::Dlpf::bypass) {
    value |= static_cast<uint8_t>((static_cast<uint8_t>(dlpf) << 3) | 0x01);
  }
  return value;
}

uint32_t magnetometerStartupDelayMs(ICM20948::MagnetometerOdr odr) {
  switch (odr) {
  case ICM20948::MagnetometerOdr::hz10:
    return 101;
  case ICM20948::MagnetometerOdr::hz20:
    return 51;
  case ICM20948::MagnetometerOdr::hz50:
    return 21;
  case ICM20948::MagnetometerOdr::hz100:
    return 11;
  case ICM20948::MagnetometerOdr::off:
    return 0;
  }
  return 0;
}

void rememberFirst(esp_err_t result, esp_err_t &first_error) {
  if (first_error == ESP_OK && result != ESP_OK)
    first_error = result;
}

int16_t signedBigEndian(const uint8_t *data) {
  int32_t value = static_cast<int32_t>((uint16_t{data[0]} << 8) | data[1]);
  if ((value & 0x8000) != 0)
    value -= 0x10000;
  return static_cast<int16_t>(value);
}

int16_t signedLittleEndian(const uint8_t *data) {
  int32_t value =
      static_cast<int32_t>(uint16_t{data[0]} | (uint16_t{data[1]} << 8));
  if ((value & 0x8000) != 0)
    value -= 0x10000;
  return static_cast<int16_t>(value);
}

} // 名前なし名前空間

ICM20948::~ICM20948() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t ICM20948::selectBank(uint8_t bank) {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (bank != kBank0 && bank != kBank2 && bank != kBank3)
    return ESP_ERR_INVALID_ARG;
  return spi_->writeRegister(device_, kRegisterBankSelect, bank);
}

esp_err_t ICM20948::magnetometerTransfer(uint8_t address, bool read,
                                         uint8_t *value, uint32_t timeout_ms) {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (value == nullptr || timeout_ms == 0)
    return ESP_ERR_INVALID_ARG;

  const auto finish = [this](esp_err_t result) {
    const esp_err_t restore = selectBank(kBank0);
    return result == ESP_OK ? restore : result;
  };

  esp_err_t result = selectBank(kBank0);
  if (result != ESP_OK)
    return result;

  uint8_t ignored_status = 0;
  result =
      spi_->readRegister(device_, kRead | kI2cMasterStatus, ignored_status);
  if (result != ESP_OK)
    return result;

  result = selectBank(kBank3);
  if (result != ESP_OK)
    return result;
  result = spi_->writeRegister(
      device_, kI2cSlave4Address,
      static_cast<uint8_t>(kMagnetometerAddress | (read ? kRead : 0)));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kI2cSlave4Register, address);
  if (result == ESP_OK && !read)
    result = spi_->writeRegister(device_, kI2cSlave4Output, *value);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kI2cSlave4Control, kI2cSlaveEnable);
  if (result != ESP_OK)
    return finish(result);

  result = selectBank(kBank0);
  if (result != ESP_OK)
    return result;

  const int64_t deadline_us = avi_micros() + int64_t{timeout_ms} * 1000;
  bool transfer_complete = false;
  while (avi_micros() < deadline_us) {
    uint8_t status = 0;
    result = spi_->readRegister(device_, kRead | kI2cMasterStatus, status);
    if (result != ESP_OK)
      return result;
    if ((status & (kI2cSlave4Nack | kI2cLostArbitration)) != 0)
      return ESP_ERR_INVALID_RESPONSE;
    if ((status & kI2cSlave4Done) != 0) {
      transfer_complete = true;
      break;
    }
    avi_delay_ms(1);
  }
  if (!transfer_complete)
    return ESP_ERR_TIMEOUT;

  if (!read)
    return ESP_OK;

  result = selectBank(kBank3);
  if (result == ESP_OK)
    result = spi_->readRegister(device_, kRead | kI2cSlave4Input, *value);
  return finish(result);
}

esp_err_t ICM20948::configureMagnetometer(MagnetometerOdr odr) {
  if (odr == MagnetometerOdr::off || !validMagnetometerOdr(odr))
    return ESP_ERR_INVALID_ARG;

  const auto finish = [this](esp_err_t result) {
    const esp_err_t restore = selectBank(kBank0);
    return result == ESP_OK ? restore : result;
  };

  esp_err_t result = selectBank(kBank0);
  if (result != ESP_OK)
    return result;

  uint8_t interrupt_config = 0;
  result = spi_->readRegister(device_, kRead | kInterruptPinConfig,
                              interrupt_config);
  if (result == ESP_OK) {
    interrupt_config &= static_cast<uint8_t>(~kBypassEnable);
    result =
        spi_->writeRegister(device_, kInterruptPinConfig, interrupt_config);
  }
  if (result == ESP_OK)
    result = selectBank(kBank3);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kI2cMasterControl, 0x07);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kI2cSlave0Control, 0x00);
  if (result != ESP_OK)
    return finish(result);

  result = selectBank(kBank0);
  if (result == ESP_OK) {
    result = spi_->writeRegister(
        device_, kUserControl,
        static_cast<uint8_t>(kI2cInterfaceDisable | kI2cMasterEnable));
  }
  if (result != ESP_OK)
    return result;

  uint8_t value = 0x01;
  result = magnetometerTransfer(kMagnetometerControl3, false, &value,
                                config_.operation_timeout_ms);
  if (result != ESP_OK)
    return result;
  avi_delay_ms(1);

  value = 0;
  result = magnetometerTransfer(kMagnetometerWhoAmI1, true, &value,
                                config_.operation_timeout_ms);
  if (result != ESP_OK)
    return result;
  if (value != kExpectedMagnetometerWhoAmI1)
    return ESP_ERR_INVALID_RESPONSE;

  result = magnetometerTransfer(kMagnetometerWhoAmI2, true, &value,
                                config_.operation_timeout_ms);
  if (result != ESP_OK)
    return result;
  if (value != kExpectedMagnetometerWhoAmI2)
    return ESP_ERR_INVALID_RESPONSE;

  value = static_cast<uint8_t>(odr);
  result = magnetometerTransfer(kMagnetometerControl2, false, &value,
                                config_.operation_timeout_ms);
  if (result != ESP_OK)
    return result;

  value = 0;
  result = magnetometerTransfer(kMagnetometerControl2, true, &value,
                                config_.operation_timeout_ms);
  if (result != ESP_OK)
    return result;
  if (value != static_cast<uint8_t>(odr))
    return ESP_ERR_INVALID_RESPONSE;

  result = selectBank(kBank3);
  if (result == ESP_OK) {
    result =
        spi_->writeRegister(device_, kI2cSlave0Address,
                            static_cast<uint8_t>(kRead | kMagnetometerAddress));
  }
  if (result == ESP_OK) {
    result =
        spi_->writeRegister(device_, kI2cSlave0Register, kMagnetometerStatus1);
  }
  if (result == ESP_OK) {
    result = spi_->writeRegister(device_, kI2cSlave0Control,
                                 static_cast<uint8_t>(kI2cSlaveEnable | 9));
  }
  result = finish(result);
  if (result != ESP_OK)
    return result;

  avi_delay_ms(magnetometerStartupDelayMs(odr));
  uint8_t master_status = 0;
  result = spi_->readRegister(device_, kRead | kI2cMasterStatus, master_status);
  if (result != ESP_OK)
    return result;
  return (master_status & kAuxiliaryI2cErrors) == 0 ? ESP_OK
                                                    : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t ICM20948::shutdownHardware(bool stop_magnetometer) {
  esp_err_t first_error = ESP_OK;

  esp_err_t result = selectBank(kBank3);
  rememberFirst(result, first_error);
  if (result == ESP_OK) {
    rememberFirst(spi_->writeRegister(device_, kI2cSlave0Control, 0x00),
                  first_error);
  }
  rememberFirst(selectBank(kBank0), first_error);

  if (stop_magnetometer) {
    uint8_t power_down = static_cast<uint8_t>(MagnetometerOdr::off);
    rememberFirst(magnetometerTransfer(kMagnetometerControl2, false,
                                       &power_down,
                                       config_.operation_timeout_ms),
                  first_error);
  }

  rememberFirst(selectBank(kBank0), first_error);
  rememberFirst(
      spi_->writeRegister(device_, kUserControl, kI2cInterfaceDisable),
      first_error);
  rememberFirst(
      spi_->writeRegister(device_, kPowerManagement1,
                          static_cast<uint8_t>(kSleep | kAutomaticClock)),
      first_error);
  return first_error;
}

esp_err_t ICM20948::begin(SPICREATE &spi, int chip_select,
                          uint32_t frequency_hz) {
  Config config{};
  config.frequency_hz = frequency_hz;
  return begin(spi, chip_select, config);
}

esp_err_t ICM20948::begin(SPICREATE &spi, int chip_select,
                          const Config &config) {
  if (spi_ != nullptr || device_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  if (!validConfig(config))
    return ESP_ERR_INVALID_ARG;

  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 0, 1}, device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;
  config_ = config;

  bool hardware_started = false;
  bool magnetometer_started = false;
  const auto fail = [this, &hardware_started,
                     &magnetometer_started](esp_err_t cause) {
    esp_err_t cleanup = ESP_OK;
    if (hardware_started)
      cleanup = shutdownHardware(magnetometer_started);
    const esp_err_t remove = spi_->removeDevice(device_);
    if (remove == ESP_OK) {
      spi_ = nullptr;
      config_ = Config{};
    }
    if (remove != ESP_OK)
      return remove;
    return cleanup == ESP_OK ? cause : cleanup;
  };

  result = selectBank(kBank0);
  if (result != ESP_OK)
    return fail(result);

  uint8_t identity = 0;
  result = spi_->readRegister(device_, kRead | kWhoAmI, identity);
  if (result != ESP_OK)
    return fail(result);
  if (identity != kExpectedWhoAmI)
    return fail(ESP_ERR_INVALID_RESPONSE);

  hardware_started = true;
  result = spi_->writeRegister(device_, kUserControl, kI2cInterfaceDisable);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kPowerManagement1, kDeviceReset);
  if (result != ESP_OK)
    return fail(result);

  avi_delay_ms(1);
  const int64_t reset_deadline =
      avi_micros() + int64_t{config.operation_timeout_ms} * 1000;
  bool reset_complete = false;
  while (avi_micros() < reset_deadline) {
    uint8_t power_management = 0;
    result = spi_->readRegister(device_, kRead | kPowerManagement1,
                                power_management);
    if (result != ESP_OK)
      return fail(result);
    if ((power_management & kDeviceReset) == 0) {
      reset_complete = true;
      break;
    }
    avi_delay_ms(1);
  }
  if (!reset_complete)
    return fail(ESP_ERR_TIMEOUT);

  result = selectBank(kBank0);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kUserControl, kI2cInterfaceDisable);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kPowerManagement1, kAutomaticClock);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kPowerManagement2, 0x00);
  if (result != ESP_OK)
    return fail(result);
  avi_delay_ms(kGyroscopeStartupDelayMs);

  result = selectBank(kBank2);
  if (result == ESP_OK) {
    result = spi_->writeRegister(device_, kGyroSampleRateDivider,
                                 config.gyro_sample_rate_divider);
  }
  if (result == ESP_OK) {
    result = spi_->writeRegister(
        device_, kGyroConfig,
        sensorConfig(static_cast<uint8_t>(config.gyro_range),
                     config.gyro_dlpf));
  }
  if (result == ESP_OK) {
    result = spi_->writeRegister(
        device_, kAccelSampleRateDividerHigh,
        static_cast<uint8_t>(config.accel_sample_rate_divider >> 8));
  }
  if (result == ESP_OK) {
    result = spi_->writeRegister(
        device_, kAccelSampleRateDividerLow,
        static_cast<uint8_t>(config.accel_sample_rate_divider));
  }
  if (result == ESP_OK) {
    result = spi_->writeRegister(
        device_, kAccelConfig,
        sensorConfig(static_cast<uint8_t>(config.accel_range),
                     config.accel_dlpf));
  }
  if (result == ESP_OK)
    result = selectBank(kBank0);
  if (result != ESP_OK)
    return fail(result);

  if (config.magnetometer_odr != MagnetometerOdr::off) {
    magnetometer_started = true;
    result = configureMagnetometer(config.magnetometer_odr);
    if (result != ESP_OK)
      return fail(result);
  }

  initialized_ = true;
  return ESP_OK;
}

esp_err_t ICM20948::end() {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;

  initialized_ = false;
  const esp_err_t shutdown =
      shutdownHardware(config_.magnetometer_odr != MagnetometerOdr::off);
  const esp_err_t remove = spi_->removeDevice(device_);
  if (remove != ESP_OK)
    return remove;

  spi_ = nullptr;
  config_ = Config{};
  return shutdown;
}

esp_err_t ICM20948::whoAmI(uint8_t &value) {
  if (!initialized_ || spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = selectBank(kBank0);
  return result == ESP_OK ? spi_->readRegister(device_, kRead | kWhoAmI, value)
                          : result;
}

esp_err_t ICM20948::getStatus(Status &status) {
  if (!initialized_ || spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;

  esp_err_t result = selectBank(kBank0);
  if (result != ESP_OK)
    return result;

  uint8_t data_ready = 0;
  result = spi_->readRegister(device_, kRead | kDataReadyStatus, data_ready);
  if (result != ESP_OK)
    return result;

  Status next{};
  next.data_ready = (data_ready & 0x01) != 0;
  next.magnetometer_enabled = config_.magnetometer_odr != MagnetometerOdr::off;
  if (next.magnetometer_enabled) {
    uint8_t master_status = 0;
    result =
        spi_->readRegister(device_, kRead | kI2cMasterStatus, master_status);
    if (result != ESP_OK)
      return result;
    next.auxiliary_i2c_error = (master_status & kAuxiliaryI2cErrors) != 0;

    uint8_t magnetic[9]{};
    result = spi_->read(device_, kRead | kExternalSensorData, magnetic,
                        sizeof(magnetic));
    if (result != ESP_OK)
      return result;
    next.magnetometer_ready = (magnetic[0] & kMagnetometerDataReady) != 0;
    next.magnetometer_overrun = (magnetic[0] & kMagnetometerOverrun) != 0;
    next.magnetometer_overflow = (magnetic[8] & kMagnetometerOverflow) != 0;
  }

  status = next;
  return ESP_OK;
}

esp_err_t ICM20948::available(bool &ready) {
  Status status{};
  const esp_err_t result = getStatus(status);
  if (result == ESP_OK)
    ready = status.data_ready;
  return result;
}

bool ICM20948::available() {
  bool ready{};
  return available(ready) == ESP_OK && ready;
}

esp_err_t ICM20948::readRaw(RawData &data) {
  if (!initialized_ || spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;

  esp_err_t result = selectBank(kBank0);
  if (result != ESP_OK)
    return result;

  uint8_t raw[14]{};
  result = spi_->read(device_, kRead | kAccelOutput, raw, sizeof(raw));
  if (result != ESP_OK)
    return result;

  uint8_t magnetic[9]{};
  if (config_.magnetometer_odr != MagnetometerOdr::off) {
    uint8_t master_status = 0;
    result =
        spi_->readRegister(device_, kRead | kI2cMasterStatus, master_status);
    if (result != ESP_OK)
      return result;
    if ((master_status & kAuxiliaryI2cErrors) != 0)
      return ESP_ERR_INVALID_RESPONSE;

    result = spi_->read(device_, kRead | kExternalSensorData, magnetic,
                        sizeof(magnetic));
    if (result != ESP_OK)
      return result;
    if ((magnetic[0] & kMagnetometerDataReady) != 0 &&
        (magnetic[8] & kMagnetometerOverflow) != 0)
      return ESP_ERR_INVALID_RESPONSE;
  }

  RawData next{};
  for (std::size_t i = 0; i < next.acceleration.size(); ++i)
    next.acceleration[i] = signedBigEndian(&raw[i * 2]);
  for (std::size_t i = 0; i < next.angular_velocity.size(); ++i)
    next.angular_velocity[i] = signedBigEndian(&raw[6 + i * 2]);
  next.temperature = signedBigEndian(&raw[12]);
  if (config_.magnetometer_odr != MagnetometerOdr::off &&
      (magnetic[0] & kMagnetometerDataReady) != 0) {
    for (std::size_t i = 0; i < next.magnetic.size(); ++i)
      next.magnetic[i] = signedLittleEndian(&magnetic[1 + i * 2]);
    next.magnetic_valid = true;
  }

  data = next;
  return ESP_OK;
}

esp_err_t ICM20948::read(Data &data) {
  RawData raw{};
  const esp_err_t result = readRaw(raw);
  if (result != ESP_OK)
    return result;

  static constexpr float kAccelSensitivity[] = {16384.0F, 8192.0F, 4096.0F,
                                                 2048.0F};
  static constexpr float kGyroSensitivity[] = {131.0F, 65.5F, 32.8F, 16.4F};
  const auto accel_index = static_cast<uint8_t>(config_.accel_range);
  const auto gyro_index = static_cast<uint8_t>(config_.gyro_range);
  Data next{};
  for (std::size_t i = 0; i < raw.acceleration.size(); ++i) {
    next.acceleration_g[i] =
        raw.acceleration[i] / kAccelSensitivity[accel_index];
    next.angular_velocity_dps[i] =
        raw.angular_velocity[i] / kGyroSensitivity[gyro_index];
    next.magnetic_ut[i] = raw.magnetic[i] * 0.15F;
  }
  next.temperature_celsius = raw.temperature / 333.87F + 21.0F;
  next.magnetic_valid = raw.magnetic_valid;
  data = next;
  return ESP_OK;
}
