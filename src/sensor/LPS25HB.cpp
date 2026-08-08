#include "LPS25HB.h"

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"

namespace {
constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kExpectedWhoAmI = 0xBD;
constexpr uint8_t kResolution = 0x10;
constexpr uint8_t kControl1 = 0x20;
constexpr uint8_t kControl2 = 0x21;
constexpr uint8_t kStatus = 0x27;
constexpr uint8_t kPressureOutput = 0x28;
constexpr uint8_t kSpiRead = 0x80;
constexpr uint8_t kSpiAutoIncrement = 0x40;
constexpr uint8_t kI2cAutoIncrement = 0x80;
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
constexpr uint32_t kMaximumI2cFrequencyHz = 400000;
constexpr int64_t kResetTimeoutUs = 100000;

bool validConfig(const LPS25HB::Config &config) {
  uint64_t timeout_ms{};
  return static_cast<uint8_t>(config.odr) <= 4 &&
         static_cast<uint8_t>(config.pressure_average) <= 3 &&
         static_cast<uint8_t>(config.temperature_average) <= 3 &&
         (config.odr != LPS25HB::Odr::one_shot ||
          (config.one_shot_timeout.isFinite() &&
           config.one_shot_timeout.millisecondsValue(timeout_ms) &&
           timeout_ms > 0));
}
} // namespace

LPS25HB::~LPS25HB() {
  if (transport_ != Transport::none)
    (void)end();
}

esp_err_t LPS25HB::begin(SPICREATE &spi, int chip_select) {
  return begin(spi, chip_select, SpiConfig{}, Config{});
}

esp_err_t LPS25HB::begin(SPICREATE &spi, int chip_select,
                         const Config &config) {
  return begin(spi, chip_select, SpiConfig{}, config);
}

esp_err_t LPS25HB::begin(SPICREATE &spi, int chip_select,
                         const SpiConfig &spi_config) {
  return begin(spi, chip_select, spi_config, Config{});
}

esp_err_t LPS25HB::begin(SPICREATE &spi, int chip_select,
                         const SpiConfig &spi_config, const Config &config) {
  if (transport_ != Transport::none)
    return ESP_ERR_INVALID_STATE;
  if (!validConfig(config) || spi_config.frequency_hz == 0 ||
      spi_config.frequency_hz > kMaximumSpiFrequencyHz)
    return ESP_ERR_INVALID_ARG;
  esp_err_t result =
      spi.addDevice({chip_select, spi_config.frequency_hz, 3, 1}, spi_device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;
  transport_ = Transport::spi;
  result = configure(config);
  if (result == ESP_OK)
    return ESP_OK;
  const esp_err_t cleanup = spi_->removeDevice(spi_device_);
  if (cleanup == ESP_OK) {
    spi_ = nullptr;
    transport_ = Transport::none;
  }
  return cleanup == ESP_OK ? result : cleanup;
}

esp_err_t LPS25HB::begin(I2CCREATE &i2c, Address address) {
  return begin(i2c, address, Config{});
}

esp_err_t LPS25HB::begin(I2CCREATE &i2c, Address address,
                         const Config &config) {
  if (transport_ != Transport::none)
    return ESP_ERR_INVALID_STATE;
  if (!validConfig(config) || i2c.frequencyHz() == 0 ||
      i2c.frequencyHz() > kMaximumI2cFrequencyHz ||
      (address != Address::low && address != Address::high))
    return ESP_ERR_INVALID_ARG;
  esp_err_t result =
      i2c.addDevice({static_cast<uint8_t>(address)}, i2c_device_);
  if (result != ESP_OK)
    return result;
  i2c_ = &i2c;
  transport_ = Transport::i2c;
  result = configure(config);
  if (result == ESP_OK)
    return ESP_OK;
  const esp_err_t cleanup = i2c_->removeDevice(i2c_device_);
  if (cleanup == ESP_OK) {
    i2c_ = nullptr;
    transport_ = Transport::none;
  }
  return cleanup == ESP_OK ? result : cleanup;
}

uint8_t LPS25HB::control2Base() const {
  return transport_ == Transport::spi ? kI2cDisable : 0;
}

esp_err_t LPS25HB::readRegister(uint8_t address, uint8_t &value) {
  if (transport_ == Transport::spi)
    return spi_->readRegister(spi_device_, kSpiRead | address, value);
  if (transport_ == Transport::i2c)
    return i2c_->readRegister(i2c_device_, address, value);
  return ESP_ERR_INVALID_STATE;
}

esp_err_t LPS25HB::writeRegister(uint8_t address, uint8_t value) {
  if (transport_ == Transport::spi)
    return spi_->writeRegister(spi_device_, address, value);
  if (transport_ == Transport::i2c)
    return i2c_->writeRegister(i2c_device_, address, value);
  return ESP_ERR_INVALID_STATE;
}

esp_err_t LPS25HB::readRegisters(uint8_t address, uint8_t *data,
                                 std::size_t length) {
  if (transport_ == Transport::spi)
    return spi_->read(spi_device_, kSpiRead | kSpiAutoIncrement | address, data,
                      length);
  if (transport_ == Transport::i2c)
    return i2c_->readRegisters(
        i2c_device_, static_cast<uint8_t>(kI2cAutoIncrement | address), data,
        length);
  return ESP_ERR_INVALID_STATE;
}

esp_err_t LPS25HB::configure(const Config &config) {
  uint8_t identity{};
  esp_err_t result = readRegister(kWhoAmI, identity);
  if (result != ESP_OK)
    return result;
  if (identity != kExpectedWhoAmI)
    return ESP_ERR_INVALID_RESPONSE;
  result = writeRegister(kControl2,
                         static_cast<uint8_t>(control2Base() | kSoftwareReset));
  if (result != ESP_OK)
    return result;
  avi_delay_ms(1);
  const int64_t deadline = avi_micros() + kResetTimeoutUs;
  while (true) {
    uint8_t control{};
    result = readRegister(kControl2, control);
    if (result != ESP_OK)
      return result;
    if ((control & kSoftwareReset) == 0)
      break;
    if (avi_micros() >= deadline)
      return ESP_ERR_TIMEOUT;
    avi_delay_ms(1);
  }
  result = writeRegister(kControl2, control2Base());
  if (result == ESP_OK) {
    const uint8_t resolution = static_cast<uint8_t>(
        (static_cast<uint8_t>(config.temperature_average) << 2) |
        static_cast<uint8_t>(config.pressure_average));
    result = writeRegister(kResolution, resolution);
  }
  uint8_t control1 = kBlockDataUpdate;
  if (config.odr != Odr::one_shot)
    control1 |=
        kPowerOn | static_cast<uint8_t>(static_cast<uint8_t>(config.odr) << 4);
  if (result == ESP_OK)
    result = writeRegister(kControl1, control1);
  if (result == ESP_OK) {
    config_ = config;
    initialized_ = true;
  }
  return result;
}

esp_err_t LPS25HB::end() {
  if (transport_ == Transport::none)
    return ESP_ERR_INVALID_STATE;
  initialized_ = false;
  const esp_err_t power_down = writeRegister(kControl1, kPowerDown);
  esp_err_t remove = ESP_ERR_INVALID_STATE;
  if (transport_ == Transport::spi)
    remove = spi_->removeDevice(spi_device_);
  else
    remove = i2c_->removeDevice(i2c_device_);
  if (remove != ESP_OK)
    return remove;
  spi_ = nullptr;
  i2c_ = nullptr;
  transport_ = Transport::none;
  config_ = Config{};
  return power_down;
}

esp_err_t LPS25HB::whoAmI(uint8_t &value) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  return readRegister(kWhoAmI, value);
}

esp_err_t LPS25HB::getStatus(Status &status) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw{};
  const esp_err_t result = readRegister(kStatus, raw);
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

esp_err_t LPS25HB::available(bool &ready) {
  Status status{};
  const esp_err_t result = getStatus(status);
  if (result == ESP_OK)
    ready = status.pressure_ready && status.temperature_ready;
  return result;
}

bool LPS25HB::available() {
  bool ready{};
  return available(ready) == ESP_OK && ready;
}

esp_err_t LPS25HB::readRaw(RawData &data) {
  if (!initialized_)
    return ESP_ERR_INVALID_STATE;
  if (config_.odr == Odr::one_shot) {
    esp_err_t result = writeRegister(
        kControl2, static_cast<uint8_t>(control2Base() | kOneShot));
    if (result != ESP_OK)
      return result;
    avi::internal::Deadline deadline{};
    result = avi::internal::makeDeadline(config_.one_shot_timeout, deadline);
    if (result != ESP_OK)
      return ESP_ERR_INVALID_ARG;
    while (true) {
      uint8_t control{};
      result = readRegister(kControl2, control);
      if (result != ESP_OK)
        return result;
      Status status{};
      result = getStatus(status);
      if (result != ESP_OK)
        return result;
      if ((control & kOneShot) == 0 && status.pressure_ready &&
          status.temperature_ready)
        break;
      if (avi::internal::expired(deadline))
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
  const esp_err_t result = readRegisters(kPressureOutput, raw, sizeof(raw));
  if (result != ESP_OK)
    return result;
  const uint32_t pressure_bits =
      uint32_t{raw[0]} | (uint32_t{raw[1]} << 8) | (uint32_t{raw[2]} << 16);
  int32_t pressure = static_cast<int32_t>(pressure_bits);
  if ((pressure_bits & 0x00800000U) != 0)
    pressure -= 0x01000000;
  int32_t temperature =
      static_cast<int32_t>(uint32_t{raw[3]} | (uint32_t{raw[4]} << 8));
  if ((temperature & 0x8000) != 0)
    temperature -= 0x10000;
  RawData next{};
  next.pressure = pressure;
  next.temperature = static_cast<int16_t>(temperature);
  data = next;
  return ESP_OK;
}

esp_err_t LPS25HB::read(Data &data) {
  RawData raw{};
  const esp_err_t result = readRaw(raw);
  if (result != ESP_OK)
    return result;
  Data next{};
  next.pressure_pa = static_cast<float>(raw.pressure) * 100.0F / 4096.0F;
  next.temperature_celsius =
      42.5F + static_cast<float>(raw.temperature) / 480.0F;
  data = next;
  return ESP_OK;
}
