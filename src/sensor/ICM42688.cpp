#include "ICM42688.h"

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <cmath>

namespace {

constexpr uint8_t kDeviceConfig = 0x11;
constexpr uint8_t kIntConfig = 0x14;
constexpr uint8_t kTemperatureData = 0x1D;
constexpr uint8_t kAccelData = 0x1F;
constexpr uint8_t kGyroData = 0x25;
constexpr uint8_t kIntStatus = 0x2D;
constexpr uint8_t kPowerManagement = 0x4E;
constexpr uint8_t kGyroConfig = 0x4F;
constexpr uint8_t kAccelConfig = 0x50;
constexpr uint8_t kGyroConfig1 = 0x51;
constexpr uint8_t kGyroAccelFilter = 0x52;
constexpr uint8_t kAccelConfig1 = 0x53;
constexpr uint8_t kIntConfig0 = 0x63;
constexpr uint8_t kIntConfig1 = 0x64;
constexpr uint8_t kIntSource0 = 0x65;
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kRegisterBankSelect = 0x76;
constexpr uint8_t kSelfTestConfig = 0x70;
constexpr uint8_t kGyroSelfTestData = 0x5F;
constexpr uint8_t kAccelSelfTestData = 0x3B;
constexpr uint8_t kExpectedWhoAmI = 0x47;
constexpr uint32_t kMaximumSpiFrequency = 24000000;
constexpr uint8_t kIntTpulseDuration = 0x40;
constexpr uint8_t kIntTdeassertDisable = 0x20;
constexpr uint8_t kIntAsyncReset = 0x10;
constexpr std::size_t kSelfTestSamples = 200;
constexpr uint8_t kGyroSelfTestAxes = 0x38;
constexpr uint8_t kAccelSelfTestAxesAndRegulator = 0x47;

bool accelBits(ICM42688::AccelRange range, uint8_t &bits) {
  switch (range) {
  case ICM42688::AccelRange::g16:
    bits = 0;
    return true;
  case ICM42688::AccelRange::g8:
    bits = 1U << 5;
    return true;
  case ICM42688::AccelRange::g4:
    bits = 2U << 5;
    return true;
  case ICM42688::AccelRange::g2:
    bits = 3U << 5;
    return true;
  }
  return false;
}

bool gyroBits(ICM42688::GyroRange range, uint8_t &bits) {
  switch (range) {
  case ICM42688::GyroRange::dps2000:
    bits = 0;
    return true;
  case ICM42688::GyroRange::dps1000:
    bits = 1U << 5;
    return true;
  case ICM42688::GyroRange::dps500:
    bits = 2U << 5;
    return true;
  case ICM42688::GyroRange::dps250:
    bits = 3U << 5;
    return true;
  case ICM42688::GyroRange::dps125:
    bits = 4U << 5;
    return true;
  case ICM42688::GyroRange::dps62_5:
    bits = 5U << 5;
    return true;
  case ICM42688::GyroRange::dps31_25:
    bits = 6U << 5;
    return true;
  case ICM42688::GyroRange::dps15_625:
    bits = 7U << 5;
    return true;
  }
  return false;
}

template <typename Odr> bool odrBits(Odr odr, uint8_t &bits) {
  switch (odr) {
  case Odr::hz32000:
    bits = 0x01;
    return true;
  case Odr::hz16000:
    bits = 0x02;
    return true;
  case Odr::hz8000:
    bits = 0x03;
    return true;
  case Odr::hz4000:
    bits = 0x04;
    return true;
  case Odr::hz2000:
    bits = 0x05;
    return true;
  case Odr::hz1000:
    bits = 0x06;
    return true;
  case Odr::hz200:
    bits = 0x07;
    return true;
  case Odr::hz100:
    bits = 0x08;
    return true;
  case Odr::hz50:
    bits = 0x09;
    return true;
  case Odr::hz25:
    bits = 0x0A;
    return true;
  case Odr::hz12_5:
    bits = 0x0B;
    return true;
  case Odr::hz500:
    bits = 0x0F;
    return true;
  }
  return false;
}

template <typename Odr> constexpr bool isHighOdr(Odr odr) {
  switch (odr) {
  case Odr::hz4000:
  case Odr::hz8000:
  case Odr::hz16000:
  case Odr::hz32000:
    return true;
  case Odr::hz12_5:
  case Odr::hz25:
  case Odr::hz50:
  case Odr::hz100:
  case Odr::hz200:
  case Odr::hz500:
  case Odr::hz1000:
  case Odr::hz2000:
    return false;
  }
  return false;
}

constexpr bool requiresHighOdrInterruptConfig(ICM42688::AccelOdr accel_odr,
                                              ICM42688::GyroOdr gyro_odr) {
  return isHighOdr(accel_odr) || isHighOdr(gyro_odr);
}

static_assert(!requiresHighOdrInterruptConfig(ICM42688::AccelOdr::hz1000,
                                              ICM42688::GyroOdr::hz2000));
static_assert(requiresHighOdrInterruptConfig(ICM42688::AccelOdr::hz4000,
                                             ICM42688::GyroOdr::hz1000));
static_assert(requiresHighOdrInterruptConfig(ICM42688::AccelOdr::hz1000,
                                             ICM42688::GyroOdr::hz32000));
static_assert(requiresHighOdrInterruptConfig(ICM42688::AccelOdr::hz8000,
                                             ICM42688::GyroOdr::hz1000));
static_assert(requiresHighOdrInterruptConfig(ICM42688::AccelOdr::hz1000,
                                             ICM42688::GyroOdr::hz16000));

bool filterBits(ICM42688::Filter filter, uint8_t &bits) {
  switch (filter) {
  case ICM42688::Filter::odr_div2:
    bits = 0;
    return true;
  case ICM42688::Filter::odr_div4:
    bits = 1;
    return true;
  case ICM42688::Filter::odr_div5:
    bits = 2;
    return true;
  case ICM42688::Filter::odr_div8:
    bits = 3;
    return true;
  case ICM42688::Filter::odr_div10:
    bits = 4;
    return true;
  case ICM42688::Filter::odr_div16:
    bits = 5;
    return true;
  case ICM42688::Filter::odr_div20:
    bits = 6;
    return true;
  case ICM42688::Filter::odr_div40:
    bits = 7;
    return true;
  }
  return false;
}

void rememberFirst(esp_err_t result, esp_err_t &first_error) {
  if (first_error == ESP_OK && result != ESP_OK)
    first_error = result;
}

int16_t signedWord(const uint8_t *data) {
  return static_cast<int16_t>((uint16_t{data[0]} << 8) | data[1]);
}

float accelSensitivity(ICM42688::AccelRange range) {
  switch (range) {
  case ICM42688::AccelRange::g2:
    return 16384.0F;
  case ICM42688::AccelRange::g4:
    return 8192.0F;
  case ICM42688::AccelRange::g8:
    return 4096.0F;
  case ICM42688::AccelRange::g16:
    return 2048.0F;
  }
  return 1.0F;
}

float gyroSensitivity(ICM42688::GyroRange range) {
  switch (range) {
  case ICM42688::GyroRange::dps125:
    return 262.0F;
  case ICM42688::GyroRange::dps62_5:
    return 524.3F;
  case ICM42688::GyroRange::dps31_25:
    return 1048.6F;
  case ICM42688::GyroRange::dps15_625:
    return 2097.2F;
  case ICM42688::GyroRange::dps250:
    return 131.0F;
  case ICM42688::GyroRange::dps500:
    return 65.5F;
  case ICM42688::GyroRange::dps1000:
    return 32.8F;
  case ICM42688::GyroRange::dps2000:
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

void ICM42688::dataReadyIsr(void *context) {
  // SAFETY: contextはICM42688::InterruptStateを指し、
  // gpio_isr_handler_remove()が成功するまで生存する。ISRでは固定セマフォの
  // 通知だけを行い、SPI通信、heap操作、logging、blockingを行わない。
  auto *state = static_cast<InterruptState *>(context);
  BaseType_t task_awoken = pdFALSE;
  (void)xSemaphoreGiveFromISR(state->signal, &task_awoken);
  if (task_awoken == pdTRUE)
    portYIELD_FROM_ISR();
}

ICM42688::~ICM42688() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t ICM42688::begin(SPICREATE &spi, int chip_select, uint32_t frequency) {
  Config config{};
  config.frequency_hz = frequency;
  return begin(spi, chip_select, config);
}

esp_err_t ICM42688::begin(SPICREATE &spi, int chip_select,
                          const Config &config) {
  if (device_ != nullptr)
    return ESP_ERR_INVALID_STATE;

  uint8_t accel_range{};
  uint8_t gyro_range{};
  uint8_t accel_odr{};
  uint8_t gyro_odr{};
  uint8_t filter{};
  if (config.frequency_hz == 0 || config.frequency_hz > kMaximumSpiFrequency ||
      (config.int_gpio != GPIO_NUM_NC &&
       !GPIO_IS_VALID_GPIO(config.int_gpio)) ||
      !accelBits(config.accel_range, accel_range) ||
      !gyroBits(config.gyro_range, gyro_range) ||
      !odrBits(config.accel_odr, accel_odr) ||
      !odrBits(config.gyro_odr, gyro_odr) || !filterBits(config.filter, filter))
    return ESP_ERR_INVALID_ARG;

  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 0, 1}, device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;

  result = spi_->writeRegister(device_, kDeviceConfig, 0x01);
  if (result == ESP_OK)
    avi_delay_ms(2);

  uint8_t identity{};
  if (result == ESP_OK)
    result = spi_->readRegister(device_, kWhoAmI | 0x80, identity);
  if (result == ESP_OK && identity != kExpectedWhoAmI)
    result = ESP_ERR_INVALID_RESPONSE;
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kGyroConfig,
                                 static_cast<uint8_t>(gyro_range | gyro_odr));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kAccelConfig,
                                 static_cast<uint8_t>(accel_range | accel_odr));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kGyroAccelFilter,
                                 static_cast<uint8_t>((filter << 4) | filter));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kPowerManagement, 0x0F);
  if (result == ESP_OK)
    avi_delay_ms(45);

  if (result == ESP_OK && config.int_gpio != GPIO_NUM_NC) {
    interrupt_.signal = xSemaphoreCreateBinaryStatic(&interrupt_.storage);
    if (interrupt_.signal == nullptr) {
      result = ESP_ERR_NO_MEM;
    } else {
      gpio_config_t gpio{};
      gpio.pin_bit_mask = uint64_t{1} << config.int_gpio;
      gpio.mode = GPIO_MODE_INPUT;
      gpio.pull_up_en = GPIO_PULLUP_DISABLE;
      gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
      gpio.intr_type = GPIO_INTR_POSEDGE;
      result = gpio_config(&gpio);
      if (result == ESP_OK) {
        result = gpio_install_isr_service(0);
        if (result == ESP_ERR_INVALID_STATE)
          result = ESP_OK;
      }
      if (result == ESP_OK)
        result =
            gpio_isr_handler_add(config.int_gpio, dataReadyIsr, &interrupt_);
      if (result == ESP_OK) {
        int_gpio_ = config.int_gpio;
      } else {
        interrupt_.signal = nullptr;
        (void)gpio_reset_pin(config.int_gpio);
      }
    }
  }

  if (result == ESP_OK && interrupt_.signal != nullptr)
    result = spi_->writeRegister(device_, kIntConfig, 0x03);
  if (result == ESP_OK && interrupt_.signal != nullptr)
    result = spi_->writeRegister(device_, kIntConfig0, 0x00);
  if (result == ESP_OK && interrupt_.signal != nullptr) {
    uint8_t int_config1{};
    result = spi_->readRegister(device_, kIntConfig1 | 0x80, int_config1);
    if (result == ESP_OK) {
      const uint8_t high_odr_bits =
          requiresHighOdrInterruptConfig(config.accel_odr, config.gyro_odr)
              ? kIntTpulseDuration | kIntTdeassertDisable
              : 0;
      int_config1 = static_cast<uint8_t>(
          (int_config1 &
           static_cast<uint8_t>(
               ~(kIntTpulseDuration | kIntTdeassertDisable | kIntAsyncReset))) |
          high_odr_bits);
      result = spi_->writeRegister(device_, kIntConfig1, int_config1);
    }
  }
  if (result == ESP_OK && interrupt_.signal != nullptr)
    result = spi_->writeRegister(device_, kIntSource0, 0x08);
  if (result == ESP_OK && interrupt_.signal != nullptr)
    result = gpio_intr_enable(int_gpio_);

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

esp_err_t ICM42688::end() {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;

  esp_err_t first_error = ESP_OK;
  if (interrupt_.signal != nullptr) {
    rememberFirst(gpio_intr_disable(int_gpio_), first_error);
    rememberFirst(spi_->writeRegister(device_, kIntSource0, 0x00), first_error);
    const esp_err_t remove_result = gpio_isr_handler_remove(int_gpio_);
    rememberFirst(remove_result, first_error);
    if (remove_result != ESP_OK)
      return first_error;
    // handlerの解除成功後にだけISR contextを無効化する。
    // GPIO ISRサービスはプロセス全体の共有資源なので、対象GPIOの
    // ハンドラだけを外す。サービス全体の解除は他コンポーネントを破壊する。
    interrupt_.signal = nullptr;
    rememberFirst(gpio_reset_pin(int_gpio_), first_error);
    int_gpio_ = GPIO_NUM_NC;
  }

  initialized_ = false;
  rememberFirst(spi_->writeRegister(device_, kPowerManagement, 0x00),
                first_error);
  const esp_err_t remove_result = spi_->removeDevice(device_);
  rememberFirst(remove_result, first_error);
  if (remove_result == ESP_OK) {
    device_ = nullptr;
    config_ = Config{};
    spi_ = nullptr;
  }
  return first_error;
}

esp_err_t ICM42688::whoAmI(uint8_t &value) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, kWhoAmI | 0x80, value);
}

esp_err_t ICM42688::getStatus(Status &status) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t value{};
  const esp_err_t result =
      spi_->readRegister(device_, kIntStatus | 0x80, value);
  if (result == ESP_OK) {
    status.data_ready = (value & 0x08) != 0;
  }
  return result;
}

esp_err_t ICM42688::available(bool &ready) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (interrupt_.signal != nullptr) {
    ready = uxSemaphoreGetCount(interrupt_.signal) != 0;
    return ESP_OK;
  }
  Status status{};
  const esp_err_t result = getStatus(status);
  if (result == ESP_OK)
    ready = status.data_ready;
  return result;
}

bool ICM42688::available() {
  bool ready{};
  return available(ready) == ESP_OK && ready;
}

esp_err_t ICM42688::waitDataReady(avi::Timeout timeout) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t ticks{};
  if (avi::internal::timeoutToTicks(timeout, ticks) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  if (interrupt_.signal != nullptr) {
    if (xSemaphoreTake(interrupt_.signal, ticks) == pdTRUE)
      return ESP_OK;
    return timeout.isNoWait() ? ESP_ERR_NOT_FINISHED : ESP_ERR_TIMEOUT;
  }

  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(timeout, deadline) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  Status status{};
  do {
    const esp_err_t result = getStatus(status);
    if (result != ESP_OK || status.data_ready)
      return result;
    if (timeout.isNoWait())
      return ESP_ERR_NOT_FINISHED;
    avi_delay_ms(1);
  } while (!avi::internal::expired(deadline));
  return ESP_ERR_TIMEOUT;
}

esp_err_t ICM42688::readRaw(RawData &data) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw[14]{};
  const esp_err_t result =
      spi_->read(device_, kTemperatureData | 0x80, raw, sizeof(raw));
  if (result != ESP_OK)
    return result;

  RawData next{};
  next.temperature = signedWord(raw);
  for (std::size_t i = 0; i < next.acceleration.size(); ++i)
    next.acceleration[i] = signedWord(&raw[2 + i * 2]);
  for (std::size_t i = 0; i < next.angular_velocity.size(); ++i)
    next.angular_velocity[i] = signedWord(&raw[8 + i * 2]);
  data = next;
  return ESP_OK;
}

esp_err_t ICM42688::read(Data &data) {
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
  next.temperature_celsius = raw.temperature / 132.48F + 25.0F;
  data = next;
  return ESP_OK;
}

esp_err_t ICM42688::selfTest(SelfTestResult &result, avi::Timeout timeout) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint64_t timeout_ms{};
  if (!timeout.isFinite() || !timeout.millisecondsValue(timeout_ms) ||
      timeout_ms == 0)
    return ESP_ERR_INVALID_ARG;

  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(timeout, deadline) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  SelfTestResult next{};
  uint8_t saved_power{};
  uint8_t saved_gyro{};
  uint8_t saved_accel{};
  uint8_t saved_gyro_config1{};
  uint8_t saved_accel_config1{};
  uint8_t saved_filter{};
  uint8_t saved_self_test{};
  esp_err_t operation =
      spi_->readRegister(device_, kPowerManagement | 0x80, saved_power);
  if (operation == ESP_OK)
    operation = spi_->readRegister(device_, kGyroConfig | 0x80, saved_gyro);
  if (operation == ESP_OK)
    operation = spi_->readRegister(device_, kAccelConfig | 0x80, saved_accel);
  if (operation == ESP_OK)
    operation =
        spi_->readRegister(device_, kGyroConfig1 | 0x80, saved_gyro_config1);
  if (operation == ESP_OK)
    operation =
        spi_->readRegister(device_, kAccelConfig1 | 0x80, saved_accel_config1);
  if (operation == ESP_OK)
    operation =
        spi_->readRegister(device_, kGyroAccelFilter | 0x80, saved_filter);
  if (operation == ESP_OK)
    operation =
        spi_->readRegister(device_, kSelfTestConfig | 0x80, saved_self_test);
  const bool registers_saved = operation == ESP_OK;

  uint8_t gyro_codes[3]{};
  uint8_t accel_codes[3]{};
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kRegisterBankSelect, 1);
  for (std::size_t i = 0; i < 3 && operation == ESP_OK; ++i)
    operation = spi_->readRegister(
        device_, static_cast<uint8_t>(kGyroSelfTestData + i) | 0x80,
        gyro_codes[i]);
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kRegisterBankSelect, 2);
  for (std::size_t i = 0; i < 3 && operation == ESP_OK; ++i)
    operation = spi_->readRegister(
        device_, static_cast<uint8_t>(kAccelSelfTestData + i) | 0x80,
        accel_codes[i]);
  const esp_err_t bank_restore =
      spi_->writeRegister(device_, kRegisterBankSelect, 0);
  if (operation == ESP_OK)
    operation = bank_restore;

  const auto collect = [this, &deadline](uint8_t address,
                                         std::array<int32_t, 3> &output) {
    std::array<int64_t, 3> sum{};
    std::size_t accepted = 0;
    while (accepted < kSelfTestSamples) {
      if (avi::internal::expired(deadline))
        return ESP_ERR_TIMEOUT;
      uint8_t int_status{};
      esp_err_t error =
          spi_->readRegister(device_, kIntStatus | 0x80, int_status);
      if (error != ESP_OK)
        return error;
      if ((int_status & 0x08) == 0) {
        avi_delay_ms(1);
        continue;
      }
      uint8_t raw[6]{};
      error = spi_->read(device_, static_cast<uint8_t>(address | 0x80), raw,
                         sizeof(raw));
      if (error != ESP_OK)
        return error;
      const int16_t values[3]{signedWord(&raw[0]), signedWord(&raw[2]),
                              signedWord(&raw[4])};
      if (values[0] == INT16_MIN || values[1] == INT16_MIN ||
          values[2] == INT16_MIN)
        continue;
      for (std::size_t axis = 0; axis < 3; ++axis)
        sum[axis] += values[axis];
      ++accepted;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
      output[axis] = static_cast<int32_t>(sum[axis] / kSelfTestSamples);
    return ESP_OK;
  };

  const auto checkedDelay = [&deadline](uint32_t milliseconds) {
    if (avi::internal::expired(deadline))
      return ESP_ERR_TIMEOUT;
    avi_delay_ms(milliseconds);
    return avi::internal::expired(deadline) ? ESP_ERR_TIMEOUT : ESP_OK;
  };

  // Gyroは±250 dps、1 kHz、約100 Hz帯域、3次filter、LNで検査する。
  if (operation == ESP_OK)
    operation =
        spi_->writeRegister(device_, kPowerManagement,
                            static_cast<uint8_t>((saved_power & 0xF0) | 0x0C));
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kGyroConfig, 0x66);
  if (operation == ESP_OK)
    operation = spi_->writeRegister(
        device_, kGyroConfig1,
        static_cast<uint8_t>((saved_gyro_config1 & 0xF0) | 0x0A));
  if (operation == ESP_OK)
    operation =
        spi_->writeRegister(device_, kGyroAccelFilter,
                            static_cast<uint8_t>((saved_filter & 0xF0) | 0x04));
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kSelfTestConfig, 0x00);
  if (operation == ESP_OK)
    operation = checkedDelay(60);
  if (operation == ESP_OK)
    operation = collect(kGyroData, next.gyro_baseline);
  if (operation == ESP_OK)
    operation =
        spi_->writeRegister(device_, kSelfTestConfig, kGyroSelfTestAxes);
  if (operation == ESP_OK)
    operation = checkedDelay(200);
  if (operation == ESP_OK)
    operation = collect(kGyroData, next.gyro_stimulated);
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kSelfTestConfig, 0x00);

  // Accelは±2 g、1 kHz、約100 Hz帯域、3次filter、LNで検査する。
  if (operation == ESP_OK)
    operation =
        spi_->writeRegister(device_, kPowerManagement,
                            static_cast<uint8_t>((saved_power & 0xF0) | 0x03));
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kAccelConfig, 0x66);
  if (operation == ESP_OK)
    operation = spi_->writeRegister(
        device_, kAccelConfig1,
        static_cast<uint8_t>((saved_accel_config1 & 0xE7) | 0x10));
  if (operation == ESP_OK)
    operation =
        spi_->writeRegister(device_, kGyroAccelFilter,
                            static_cast<uint8_t>((saved_filter & 0x0F) | 0x40));
  if (operation == ESP_OK)
    operation = checkedDelay(25);
  if (operation == ESP_OK)
    operation = collect(kAccelData, next.accel_baseline);
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kSelfTestConfig,
                                    kAccelSelfTestAxesAndRegulator);
  if (operation == ESP_OK)
    operation = checkedDelay(25);
  if (operation == ESP_OK)
    operation = collect(kAccelData, next.accel_stimulated);
  if (operation == ESP_OK)
    operation = spi_->writeRegister(device_, kSelfTestConfig, 0x00);

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

  esp_err_t restore = ESP_OK;
  const auto restoreRegister = [this, &restore](uint8_t address,
                                                uint8_t value) {
    const esp_err_t error = spi_->writeRegister(device_, address, value);
    if (restore == ESP_OK && error != ESP_OK)
      restore = error;
  };
  restoreRegister(kRegisterBankSelect, 0);
  if (registers_saved) {
    restoreRegister(kSelfTestConfig, saved_self_test);
    restoreRegister(kGyroConfig, saved_gyro);
    restoreRegister(kAccelConfig, saved_accel);
    restoreRegister(kGyroConfig1, saved_gyro_config1);
    restoreRegister(kAccelConfig1, saved_accel_config1);
    restoreRegister(kGyroAccelFilter, saved_filter);
    restoreRegister(kPowerManagement, saved_power);
  }
  if (interrupt_.signal != nullptr)
    (void)xSemaphoreTake(interrupt_.signal, 0);
  next.restored = restore == ESP_OK;
  result = next;
  return restore != ESP_OK ? restore : operation;
}
