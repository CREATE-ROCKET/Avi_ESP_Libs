#include "ICM42688.h"

#include "avi_esp_libs/compatibility.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <climits>

namespace {

constexpr uint8_t kDeviceConfig = 0x11;
constexpr uint8_t kIntConfig = 0x14;
constexpr uint8_t kTemperatureData = 0x1D;
constexpr uint8_t kIntStatus = 0x2D;
constexpr uint8_t kPowerManagement = 0x4E;
constexpr uint8_t kGyroConfig = 0x4F;
constexpr uint8_t kAccelConfig = 0x50;
constexpr uint8_t kGyroAccelFilter = 0x52;
constexpr uint8_t kIntConfig0 = 0x63;
constexpr uint8_t kIntConfig1 = 0x64;
constexpr uint8_t kIntSource0 = 0x65;
constexpr uint8_t kWhoAmI = 0x75;
constexpr uint8_t kExpectedWhoAmI = 0x47;
constexpr uint32_t kMaximumSpiFrequency = 24000000;

bool timeoutToTicks(uint32_t timeout_ms, TickType_t &ticks) {
  if (timeout_ms > INT_MAX)
    return false;
  uint64_t value = (uint64_t{timeout_ms} * configTICK_RATE_HZ + 999U) / 1000U;
  if (timeout_ms != 0 && value == 0)
    value = 1;
  if (value >= portMAX_DELAY)
    return false;
  ticks = static_cast<TickType_t>(value);
  return true;
}

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
  }
  return false;
}

bool odrBits(ICM42688::Odr odr, uint8_t &bits) {
  switch (odr) {
  case ICM42688::Odr::hz25:
    bits = 0x0A;
    return true;
  case ICM42688::Odr::hz50:
    bits = 0x09;
    return true;
  case ICM42688::Odr::hz100:
    bits = 0x08;
    return true;
  case ICM42688::Odr::hz200:
    bits = 0x07;
    return true;
  case ICM42688::Odr::hz500:
    bits = 0x0F;
    return true;
  case ICM42688::Odr::hz1000:
    bits = 0x06;
    return true;
  }
  return false;
}

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

} // 名前なし名前空間

void ICM42688::dataReadyIsr(void *context) {
  // SAFETY: contextはgpio_isr_handler_remove()が成功するまで生存する
  // ICM42688::InterruptStateである。ISRでは固定セマフォの通知だけを行う。
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
  uint8_t odr{};
  uint8_t filter{};
  if (config.frequency_hz == 0 || config.frequency_hz > kMaximumSpiFrequency ||
      (config.int_gpio != GPIO_NUM_NC &&
       !GPIO_IS_VALID_GPIO(config.int_gpio)) ||
      !accelBits(config.accel_range, accel_range) ||
      !gyroBits(config.gyro_range, gyro_range) || !odrBits(config.odr, odr) ||
      !filterBits(config.filter, filter))
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
                                 static_cast<uint8_t>(gyro_range | odr));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kAccelConfig,
                                 static_cast<uint8_t>(accel_range | odr));
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
    if (result == ESP_OK)
      result = spi_->writeRegister(
          device_, kIntConfig1,
          static_cast<uint8_t>(int_config1 & static_cast<uint8_t>(~0x10U)));
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
    if (status.data_ready && interrupt_.signal != nullptr) {
      (void)xSemaphoreTake(interrupt_.signal, 0);
    }
  }
  return result;
}

esp_err_t ICM42688::available(bool &ready) {
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

esp_err_t ICM42688::waitDataReady(uint32_t timeout_ms) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  TickType_t ticks{};
  if (!timeoutToTicks(timeout_ms, ticks))
    return ESP_ERR_INVALID_ARG;

  Status status{};
  esp_err_t result = getStatus(status);
  if (result != ESP_OK || status.data_ready)
    return result;

  if (interrupt_.signal != nullptr) {
    return xSemaphoreTake(interrupt_.signal, ticks) == pdTRUE ? ESP_OK
                                                              : ESP_ERR_TIMEOUT;
  }

  const int64_t deadline = avi_micros() + int64_t{timeout_ms} * 1000;
  do {
    if (timeout_ms == 0)
      break;
    avi_delay_ms(1);
    result = getStatus(status);
    if (result != ESP_OK || status.data_ready)
      return result;
  } while (avi_micros() < deadline);
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
