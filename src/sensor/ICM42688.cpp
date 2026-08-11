#include "ICM42688.h"

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <algorithm>
#include <climits>
#include <cmath>

namespace {

constexpr uint8_t kDeviceConfig = 0x11;
constexpr uint8_t kIntConfig = 0x14;
constexpr uint8_t kFifoConfig = 0x16;
constexpr uint8_t kTemperatureData = 0x1D;
constexpr uint8_t kAccelData = 0x1F;
constexpr uint8_t kGyroData = 0x25;
constexpr uint8_t kIntStatus = 0x2D;
constexpr uint8_t kFifoCountHigh = 0x2E;
constexpr uint8_t kFifoData = 0x30;
constexpr uint8_t kSignalPathReset = 0x4B;
constexpr uint8_t kInterfaceConfig0 = 0x4C;
constexpr uint8_t kPowerManagement = 0x4E;
constexpr uint8_t kGyroConfig = 0x4F;
constexpr uint8_t kAccelConfig = 0x50;
constexpr uint8_t kGyroConfig1 = 0x51;
constexpr uint8_t kGyroAccelFilter = 0x52;
constexpr uint8_t kAccelConfig1 = 0x53;
constexpr uint8_t kTimestampConfig = 0x54;
constexpr uint8_t kFifoConfig1 = 0x5F;
constexpr uint8_t kFifoConfig2 = 0x60;
constexpr uint8_t kFifoConfig3 = 0x61;
constexpr uint8_t kIntConfig0 = 0x63;
constexpr uint8_t kIntConfig1 = 0x64;
constexpr uint8_t kIntSource0 = 0x65;
constexpr uint8_t kFifoLostPacketLow = 0x6C;
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
constexpr uint8_t kFifoThresholdInterrupt = 0x04;
constexpr uint8_t kFifoFullInterrupt = 0x02;
constexpr uint8_t kDataReadyInterrupt = 0x08;
constexpr uint8_t kFifoFlush = 0x02;
constexpr uint8_t kFifoStreamMode = 0x40;
constexpr uint8_t kFifoPacket3Config = 0x47;
constexpr uint8_t kTimestampInternalDeltaConfig = 0x05;
constexpr uint8_t kFifoCountAndBigEndian = 0x70;
constexpr std::size_t kMaximumFifoBytes = 2080;
constexpr std::size_t kMinimumFifoPacketSize = 8;
constexpr std::size_t kFifoPacket3Size = 16;
constexpr std::size_t kMaximumFifoPacketSize = 20;
constexpr std::size_t kMaximumFifoFrames =
    kMaximumFifoBytes / kMinimumFifoPacketSize;
constexpr std::size_t kFifoHeaderOffset = 0;
constexpr std::size_t kFifoAccelOffset = 1;
constexpr std::size_t kFifoGyroOffset = 7;
constexpr std::size_t kFifoTemperatureOffset = 13;
constexpr std::size_t kFifoTimestampOffset = 14;
constexpr uint16_t kMaximumWatermarkRecords = 0x0FFF;
constexpr std::size_t kSelfTestSamples = 200;
constexpr uint8_t kSelfTestRegulatorEnable = 0x40;
constexpr uint8_t kAccelSelfTestZ = 0x20;
constexpr uint8_t kAccelSelfTestY = 0x10;
constexpr uint8_t kAccelSelfTestX = 0x08;
constexpr uint8_t kGyroSelfTestZ = 0x04;
constexpr uint8_t kGyroSelfTestY = 0x02;
constexpr uint8_t kGyroSelfTestX = 0x01;
constexpr uint8_t kGyroSelfTestAxes =
    kGyroSelfTestX | kGyroSelfTestY | kGyroSelfTestZ;
constexpr uint8_t kAccelSelfTestAxesAndRegulator =
    kSelfTestRegulatorEnable | kAccelSelfTestX | kAccelSelfTestY |
    kAccelSelfTestZ;

static_assert(kGyroSelfTestAxes == 0x07);
static_assert(kAccelSelfTestAxesAndRegulator == 0x78);

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

template <typename Odr> constexpr bool fifoOdrSupported(Odr odr) {
  switch (odr) {
  case Odr::hz25:
  case Odr::hz50:
  case Odr::hz100:
  case Odr::hz200:
  case Odr::hz500:
  case Odr::hz1000:
  case Odr::hz2000:
    return true;
  case Odr::hz12_5:
  case Odr::hz4000:
  case Odr::hz8000:
  case Odr::hz16000:
  case Odr::hz32000:
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
static_assert(!fifoOdrSupported(ICM42688::AccelOdr::hz12_5));
static_assert(fifoOdrSupported(ICM42688::AccelOdr::hz25));
static_assert(fifoOdrSupported(ICM42688::AccelOdr::hz1000));
static_assert(fifoOdrSupported(ICM42688::AccelOdr::hz2000));
static_assert(!fifoOdrSupported(ICM42688::AccelOdr::hz4000));
static_assert(!fifoOdrSupported(ICM42688::GyroOdr::hz12_5));
static_assert(fifoOdrSupported(ICM42688::GyroOdr::hz25));
static_assert(fifoOdrSupported(ICM42688::GyroOdr::hz1000));
static_assert(fifoOdrSupported(ICM42688::GyroOdr::hz2000));
static_assert(!fifoOdrSupported(ICM42688::GyroOdr::hz32000));

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

constexpr int16_t signedWord(const uint8_t *data) {
  return static_cast<int16_t>((uint16_t{data[0]} << 8) | data[1]);
}

constexpr bool validFifoHeader(uint8_t header) {
  // Packet 3、16-bit、ODR timestamp、FSYNCなし。下位2bitはODR change。
  return (header & 0xFC) == 0x68;
}

constexpr bool validFifoSample(int16_t value) { return value != INT16_MIN; }

constexpr float fifoTemperatureCelsius(int8_t value) {
  return static_cast<float>(value) / 2.07F + 25.0F;
}

constexpr uint64_t accumulateTimestamp(uint16_t ticks, uint64_t &microseconds,
                                       uint8_t &remainder) {
  // v1.6 12.7: internal clock、TMST_RES=0ではFIFO deltaへ32/30補正する。
  const uint32_t numerator = static_cast<uint32_t>(ticks) * 32U + remainder;
  microseconds += numerator / 30U;
  remainder = static_cast<uint8_t>(numerator % 30U);
  return microseconds;
}

constexpr std::size_t fifoReadRecordCount(std::size_t available,
                                          std::size_t capacity,
                                          std::size_t max_transfer_size) {
  if (max_transfer_size < kFifoPacket3Size)
    return 0;
  const std::size_t transfer_records = max_transfer_size / kFifoPacket3Size;
  const std::size_t buffer_records = kMaximumFifoBytes / kFifoPacket3Size;
  return std::min(std::min(available, capacity),
                  std::min(transfer_records, buffer_records));
}

constexpr bool fifoReady(const ICM42688::FifoStatus &status,
                         uint16_t watermark_records) {
  return status.threshold || status.full ||
         status.records_available >= watermark_records;
}

constexpr bool fifoContinuityLost(uint16_t baseline, uint16_t before,
                                  uint16_t after) {
  return before != baseline || after != baseline || after != before;
}

constexpr esp_err_t parseFifoPacket3(const uint8_t *packet,
                                     ICM42688::FifoRawData &data) {
  if (packet == nullptr)
    return ESP_ERR_INVALID_ARG;
  if (!validFifoHeader(packet[kFifoHeaderOffset]))
    return ESP_ERR_INVALID_RESPONSE;
  ICM42688::FifoRawData next{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    next.acceleration[axis] = signedWord(&packet[kFifoAccelOffset + axis * 2]);
    next.angular_velocity[axis] =
        signedWord(&packet[kFifoGyroOffset + axis * 2]);
  }
  next.temperature = static_cast<int8_t>(packet[kFifoTemperatureOffset]);
  next.timestamp_ticks =
      static_cast<uint16_t>((uint16_t{packet[kFifoTimestampOffset]} << 8) |
                            packet[kFifoTimestampOffset + 1]);
  next.acceleration_valid = validFifoSample(next.acceleration[0]) &&
                            validFifoSample(next.acceleration[1]) &&
                            validFifoSample(next.acceleration[2]);
  next.angular_velocity_valid = validFifoSample(next.angular_velocity[0]) &&
                                validFifoSample(next.angular_velocity[1]) &&
                                validFifoSample(next.angular_velocity[2]);
  next.temperature_valid = true;
  next.accel_odr_changed = (packet[kFifoHeaderOffset] & 0x02) != 0;
  next.gyro_odr_changed = (packet[kFifoHeaderOffset] & 0x01) != 0;
  data = next;
  return ESP_OK;
}

static_assert(kMaximumFifoBytes == 2080);
static_assert(kMaximumFifoPacketSize <= kMaximumFifoBytes);
static_assert(kMaximumFifoFrames == 260);
static_assert(kMaximumFifoBytes / kFifoPacket3Size == 130);
static_assert(kMaximumFifoBytes / kMaximumFifoPacketSize == 104);
static_assert(validFifoHeader(0x68));
static_assert(validFifoHeader(0x69));
static_assert(validFifoHeader(0x6A));
static_assert(validFifoHeader(0x6B));
static_assert(!validFifoHeader(0xE8));
static_assert(!validFifoHeader(0x48));
static_assert(!validFifoHeader(0x28));
static_assert(!validFifoHeader(0x78));
static_assert(!validFifoHeader(0x6C));
static_assert(!validFifoSample(INT16_MIN));
static_assert(validFifoSample(INT16_MIN + 1));
static_assert(fifoTemperatureCelsius(0) == 25.0F);
constexpr bool timestampTest() {
  uint64_t us{};
  uint8_t remainder{};
  accumulateTimestamp(937, us, remainder);
  accumulateTimestamp(938, us, remainder);
  return us == 2000 && remainder == 0;
}
static_assert(timestampTest());
constexpr bool timestampOneSecondTest() {
  uint64_t us{};
  uint8_t remainder{};
  for (std::size_t sample = 0; sample < 1000; ++sample)
    accumulateTimestamp((sample & 1U) == 0 ? 937 : 938, us, remainder);
  return us == 1'000'000 && remainder == 0;
}
static_assert(timestampOneSecondTest());
constexpr bool timestampBoundaryTest() {
  uint64_t us{};
  uint8_t remainder{};
  return accumulateTimestamp(UINT16_MAX, us, remainder) == 69904 &&
         remainder == 0;
}
static_assert(timestampBoundaryTest());
constexpr bool packetDecodeTest() {
  const uint8_t packet[kFifoPacket3Size]{0x6B, 0x00, 0x64, 0xFF, 0x9C, 0x7F,
                                         0xFF, 0x80, 0x00, 0x00, 0x02, 0xFF,
                                         0xFE, 0xFE, 0x03, 0xAA};
  ICM42688::FifoRawData data{};
  return parseFifoPacket3(packet, data) == ESP_OK &&
         data.acceleration[0] == 100 && data.acceleration[1] == -100 &&
         data.acceleration[2] == 32767 &&
         data.angular_velocity[0] == INT16_MIN &&
         data.angular_velocity[1] == 2 && data.angular_velocity[2] == -2 &&
         data.temperature == -2 && data.timestamp_ticks == 0x03AA &&
         data.acceleration_valid && !data.angular_velocity_valid &&
         data.accel_odr_changed && data.gyro_odr_changed;
}
static_assert(packetDecodeTest());
static_assert(fifoReadRecordCount(4, 8, 64) == 4);
static_assert(fifoReadRecordCount(8, 4, 64) == 4);
static_assert(fifoReadRecordCount(8, 8, 48) == 3);
static_assert(fifoReadRecordCount(8, 8, 47) == 2);
static_assert(fifoReadRecordCount(1, 1, 15) == 0);
static_assert(fifoReadRecordCount(1, 1, 16) == 1);
static_assert(!fifoReady({3, false, false, 0, false}, 4));
static_assert(fifoReady({4, false, false, 0, false}, 4));
static_assert(fifoReady({0, true, false, 0, false}, 4));
static_assert(fifoReady({0, false, true, 0, false}, 4));
static_assert(!fifoContinuityLost(0, 0, 0));
static_assert(!fifoContinuityLost(5, 5, 5));
static_assert(fifoContinuityLost(0, 1, 1));
static_assert(fifoContinuityLost(0, 0, 1));
static_assert(fifoContinuityLost(3, 3, 4));

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
  return measured > trim * 0.5F && measured < trim * 1.5F;
}

constexpr bool accelFallbackPass(float measured) {
  return measured >= 50.0F * 16384.0F / 1000.0F &&
         measured <= 1200.0F * 16384.0F / 1000.0F;
}

constexpr bool gyroFallbackPass(float measured) {
  return measured >= 60.0F * 131.0F;
}

constexpr bool gyroOffsetPass(float baseline) {
  return baseline <= 20.0F * 131.0F;
}

constexpr float magnitude(float value) { return value < 0.0F ? -value : value; }

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
static_assert(gyroOtpPass(1000.0F, 1000.0F));
static_assert(!gyroOtpPass(500.0F, 1000.0F));
static_assert(!gyroOtpPass(1500.0F, 1000.0F));
static_assert(magnitude(-1000.0F) == 1000.0F);
static_assert(accelFallbackPass(50.0F * 16384.0F / 1000.0F));
static_assert(accelFallbackPass(1200.0F * 16384.0F / 1000.0F));
static_assert(gyroFallbackPass(60.0F * 131.0F));
static_assert(gyroOffsetPass(20.0F * 131.0F));
static_assert(!gyroOffsetPass(20.0F * 131.0F + 1.0F));

} // namespace

void ICM42688::interruptIsr(void *context) {
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
      !odrBits(config.gyro_odr, gyro_odr) ||
      !filterBits(config.filter, filter) ||
      (config.fifo.enabled &&
       (config.fifo.watermark_records == 0 ||
        config.fifo.watermark_records > kMaximumWatermarkRecords ||
        accel_odr != gyro_odr || !fifoOdrSupported(config.accel_odr) ||
        !fifoOdrSupported(config.gyro_odr) ||
        spi.maxTransferSize() < kFifoPacket3Size)))
    return ESP_ERR_INVALID_ARG;

  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 0, 1}, device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;
  // begin途中のcleanupでも、設定済みFIFOを安全に停止できるよう保持する。
  config_ = config;

  result = spi_->writeRegister(device_, kDeviceConfig, 0x01);
  if (result == ESP_OK)
    avi_delay_ms(2);

  uint8_t identity{};
  if (result == ESP_OK)
    result = spi_->readRegister(device_, kWhoAmI | 0x80, identity);
  if (result == ESP_OK && identity != kExpectedWhoAmI)
    result = ESP_ERR_INVALID_RESPONSE;
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
      gpio.intr_type = GPIO_INTR_DISABLE;
      result = gpio_config(&gpio);
      if (result == ESP_OK) {
        result = gpio_install_isr_service(0);
        if (result == ESP_ERR_INVALID_STATE)
          result = ESP_OK;
      }
      if (result == ESP_OK)
        result =
            gpio_isr_handler_add(config.int_gpio, interruptIsr, &interrupt_);
      if (result == ESP_OK) {
        int_gpio_ = config.int_gpio;
      } else {
        interrupt_.signal = nullptr;
        (void)gpio_reset_pin(config.int_gpio);
      }
    }
  }

  const auto updateRegister = [this](uint8_t address, uint8_t clear_mask,
                                     uint8_t set_mask) {
    uint8_t value{};
    esp_err_t error = spi_->readRegister(
        device_, static_cast<uint8_t>(address | 0x80), value);
    if (error == ESP_OK) {
      value = static_cast<uint8_t>((value & ~clear_mask) | set_mask);
      error = spi_->writeRegister(device_, address, value);
    }
    return error;
  };

  // v1.6 12.9に従い、sensorがOFFの間に全configurationを完了する。
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kGyroConfig,
                                 static_cast<uint8_t>(gyro_range | gyro_odr));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kAccelConfig,
                                 static_cast<uint8_t>(accel_range | accel_odr));
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kGyroAccelFilter,
                                 static_cast<uint8_t>((filter << 4) | filter));
  if (result == ESP_OK && interrupt_.signal != nullptr)
    result = updateRegister(kIntConfig, 0x07, 0x03);
  if (result == ESP_OK && interrupt_.signal != nullptr)
    result = updateRegister(kIntConfig0, 0x3F, 0x00);
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
  if (result == ESP_OK && config.fifo.enabled)
    result = updateRegister(kInterfaceConfig0, 0xF0, kFifoCountAndBigEndian);
  if (result == ESP_OK && config.fifo.enabled)
    result =
        updateRegister(kTimestampConfig, 0x0F, kTimestampInternalDeltaConfig);
  if (result == ESP_OK && config.fifo.enabled)
    result = updateRegister(kFifoConfig1, 0x7F, kFifoPacket3Config);
  if (result == ESP_OK && config.fifo.enabled)
    result = spi_->writeRegister(
        device_, kFifoConfig2,
        static_cast<uint8_t>(config.fifo.watermark_records));
  if (result == ESP_OK && config.fifo.enabled)
    result = updateRegister(
        kFifoConfig3, 0x0F,
        static_cast<uint8_t>(config.fifo.watermark_records >> 8));
  if (result == ESP_OK)
    result = updateRegister(
        kIntSource0,
        kDataReadyInterrupt | kFifoThresholdInterrupt | kFifoFullInterrupt,
        interrupt_.signal == nullptr
            ? 0
            : (config.fifo.enabled
                   ? kFifoThresholdInterrupt | kFifoFullInterrupt
                   : kDataReadyInterrupt));
  if (result == ESP_OK && config.fifo.enabled)
    result = updateRegister(kFifoConfig, 0xC0, kFifoStreamMode);
  if (result == ESP_OK && config.fifo.enabled)
    result = spi_->writeRegister(device_, kSignalPathReset, kFifoFlush);
  uint8_t pending{};
  if (result == ESP_OK)
    result = spi_->readRegister(device_, kIntStatus | 0x80, pending);
  if (result == ESP_OK)
    result = spi_->writeRegister(device_, kPowerManagement, 0x0F);
  if (result == ESP_OK) {
    // v1.6 14.36はON遷移後200usのregister write禁止とgyro 45ms起動を規定する。
    avi_delay_ms(45);
  }
  if (result == ESP_OK && config.fifo.enabled)
    result = drainFifo();
  if (result == ESP_OK) {
    resetFifoState();
    if (config.fifo.enabled)
      result = readFifoLostPackets(fifo_lost_packets_baseline_);
  }
  if (result == ESP_OK) {
    result = spi_->readRegister(device_, kIntStatus | 0x80, pending);
  }
  if (result == ESP_OK && interrupt_.signal != nullptr) {
    (void)xSemaphoreTake(interrupt_.signal, 0);
    result = gpio_set_intr_type(int_gpio_, GPIO_INTR_POSEDGE);
  }
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
  }

  initialized_ = false;
  rememberFirst(spi_->writeRegister(device_, kPowerManagement, 0x00),
                first_error);
  // v1.6 14.36のOFF遷移後にconfiguration registerへ触れる。
  avi_delay_ms(1);
  uint8_t value{};
  esp_err_t operation = spi_->readRegister(device_, kIntSource0 | 0x80, value);
  if (operation == ESP_OK) {
    value = static_cast<uint8_t>(
        value &
        ~(kDataReadyInterrupt | kFifoThresholdInterrupt | kFifoFullInterrupt));
    operation = spi_->writeRegister(device_, kIntSource0, value);
  }
  rememberFirst(operation, first_error);
  if (config_.fifo.enabled) {
    operation = spi_->readRegister(device_, kFifoConfig | 0x80, value);
    if (operation == ESP_OK) {
      value = static_cast<uint8_t>(value & ~uint8_t{0xC0});
      operation = spi_->writeRegister(device_, kFifoConfig, value);
    }
    rememberFirst(operation, first_error);
    rememberFirst(spi_->writeRegister(device_, kSignalPathReset, kFifoFlush),
                  first_error);
  }

  if (interrupt_.signal != nullptr) {
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

  const esp_err_t remove_result = spi_->removeDevice(device_);
  rememberFirst(remove_result, first_error);
  if (remove_result == ESP_OK) {
    device_ = nullptr;
    config_ = Config{};
    resetFifoState();
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
  if (interrupt_.signal != nullptr && !config_.fifo.enabled) {
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

  if (interrupt_.signal != nullptr && !config_.fifo.enabled) {
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

void ICM42688::resetFifoState() {
  fifo_timestamp_us_ = 0;
  fifo_timestamp_remainder_ = 0;
  fifo_lost_packets_baseline_ = 0;
  fifo_faulted_ = false;
}

esp_err_t ICM42688::readFifoCount(uint16_t &records) {
  uint8_t raw[2]{};
  // v1.6 14.22: High読出し時にHigh/Lowが同時にlatchされるため、
  // FIFO_COUNTHから2 byteを1 transactionで読む。
  const esp_err_t result =
      spi_->read(device_, kFifoCountHigh | 0x80, raw, sizeof(raw));
  if (result == ESP_OK)
    records = static_cast<uint16_t>((uint16_t{raw[0]} << 8) | raw[1]);
  return result;
}

esp_err_t ICM42688::readFifoLostPackets(uint16_t &lost_packets) {
  uint8_t raw[2]{};
  const esp_err_t result =
      spi_->read(device_, kFifoLostPacketLow | 0x80, raw, sizeof(raw));
  if (result == ESP_OK) {
    // v1.6の一覧表と詳細欄が矛盾するため、14.55/14.56の詳細記述を採用する。
    lost_packets =
        static_cast<uint16_t>(uint16_t{raw[0]} | (uint16_t{raw[1]} << 8));
  }
  return result;
}

esp_err_t ICM42688::readFifoBytes(std::size_t capacity, std::size_t &records) {
  records = 0;
  uint16_t lost_before{};
  esp_err_t result = readFifoLostPackets(lost_before);
  if (result != ESP_OK)
    return result;
  if (lost_before != fifo_lost_packets_baseline_) {
    fifo_faulted_ = true;
    return ESP_ERR_INVALID_RESPONSE;
  }
  uint16_t available_records{};
  result = readFifoCount(available_records);
  if (result != ESP_OK)
    return result;
  const std::size_t next_records =
      fifoReadRecordCount(available_records, capacity, spi_->maxTransferSize());
  if (spi_->maxTransferSize() < kFifoPacket3Size)
    return ESP_ERR_INVALID_STATE;
  if (next_records != 0) {
    // FIFO packet途中でCSを切らず、16 byte record単位でburst readする。
    result = spi_->read(device_, kFifoData | 0x80, fifo_buffer_.data(),
                        next_records * kFifoPacket3Size);
    if (result != ESP_OK) {
      // 消費量を確定できないため、次のtimestampを連続とは扱えない。
      fifo_faulted_ = true;
      return result;
    }
  }
  uint16_t lost_after{};
  result = readFifoLostPackets(lost_after);
  if (result != ESP_OK) {
    if (next_records != 0)
      fifo_faulted_ = true;
    return result;
  }
  if (fifoContinuityLost(fifo_lost_packets_baseline_, lost_before,
                         lost_after)) {
    // 読み出したbatchは時系列連続性を保証できないためcallerへ渡さない。
    fifo_faulted_ = true;
    return ESP_ERR_INVALID_RESPONSE;
  }
  records = next_records;
  return ESP_OK;
}

esp_err_t ICM42688::drainFifo() {
  uint16_t available_records{};
  esp_err_t result = readFifoCount(available_records);
  std::size_t remaining = available_records;
  const std::size_t transfer_records =
      std::min(spi_->maxTransferSize(), fifo_buffer_.size()) / kFifoPacket3Size;
  if (result == ESP_OK && transfer_records == 0)
    return ESP_ERR_INVALID_STATE;
  // startup中のsampleはgyroの起動保証前なので、通常sampleとして返さない。
  while (result == ESP_OK && remaining != 0) {
    const std::size_t chunk = std::min(remaining, transfer_records);
    result = spi_->read(device_, kFifoData | 0x80, fifo_buffer_.data(),
                        chunk * kFifoPacket3Size);
    remaining -= chunk;
  }
  return result;
}

esp_err_t ICM42688::getFifoStatus(FifoStatus &status) {
  if (!initialized_ || spi_ == nullptr || !config_.fifo.enabled)
    return ESP_ERR_INVALID_STATE;
  uint8_t interrupt_status{};
  uint16_t records{};
  uint16_t lost_packets{};
  esp_err_t result =
      spi_->readRegister(device_, kIntStatus | 0x80, interrupt_status);
  if (result == ESP_OK)
    result = readFifoCount(records);
  if (result == ESP_OK)
    result = readFifoLostPackets(lost_packets);
  if (result != ESP_OK)
    return result;
  FifoStatus next{};
  next.records_available = records;
  next.threshold = (interrupt_status & kFifoThresholdInterrupt) != 0;
  next.full = (interrupt_status & kFifoFullInterrupt) != 0;
  next.lost_packets = lost_packets;
  if (lost_packets != fifo_lost_packets_baseline_)
    fifo_faulted_ = true;
  next.faulted = fifo_faulted_;
  status = next;
  return ESP_OK;
}

esp_err_t ICM42688::fifoAvailable(std::size_t &records) {
  if (!initialized_ || spi_ == nullptr || !config_.fifo.enabled)
    return ESP_ERR_INVALID_STATE;
  if (fifo_faulted_)
    return ESP_ERR_INVALID_STATE;
  uint16_t available_records{};
  const esp_err_t result = readFifoCount(available_records);
  if (result == ESP_OK)
    records = available_records;
  return result;
}

esp_err_t ICM42688::waitFifo(avi::Timeout timeout) {
  if (!initialized_ || spi_ == nullptr || !config_.fifo.enabled)
    return ESP_ERR_INVALID_STATE;
  if (fifo_faulted_)
    return ESP_ERR_INVALID_STATE;
  avi::internal::Deadline deadline{};
  if (avi::internal::makeDeadline(timeout, deadline) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  for (;;) {
    // FIFO_COUNTが真の状態であり、semaphoreは状態変化のwake-up hintに過ぎない。
    FifoStatus status{};
    const esp_err_t result = getFifoStatus(status);
    if (result != ESP_OK)
      return result;
    if (status.faulted)
      return ESP_ERR_INVALID_RESPONSE;
    if (fifoReady(status, config_.fifo.watermark_records))
      return ESP_OK;
    if (timeout.isNoWait())
      return ESP_ERR_NOT_FINISHED;
    if (avi::internal::expired(deadline))
      return ESP_ERR_TIMEOUT;

    if (interrupt_.signal == nullptr) {
      avi_delay_ms(1);
      continue;
    }

    TickType_t ticks = portMAX_DELAY;
    if (!deadline.forever) {
      const int64_t now = avi_micros();
      if (now >= deadline.microseconds)
        return ESP_ERR_TIMEOUT;
      const uint64_t remaining_us =
          static_cast<uint64_t>(deadline.microseconds - now);
      const avi::Timeout remaining =
          avi::Timeout::milliseconds((remaining_us + 999U) / 1000U);
      if (avi::internal::timeoutToTicks(remaining, ticks) != ESP_OK)
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(interrupt_.signal, ticks) != pdTRUE)
      return ESP_ERR_TIMEOUT;
  }
}

esp_err_t ICM42688::readFifoRaw(FifoRawData *data, std::size_t capacity,
                                std::size_t &count) {
  count = 0;
  if (!initialized_ || spi_ == nullptr || !config_.fifo.enabled)
    return ESP_ERR_INVALID_STATE;
  if (fifo_faulted_)
    return ESP_ERR_INVALID_STATE;
  if (data == nullptr && capacity != 0)
    return ESP_ERR_INVALID_ARG;
  if (capacity == 0) {
    count = 0;
    return ESP_OK;
  }
  std::size_t next_count{};
  esp_err_t result = readFifoBytes(capacity, next_count);
  if (result != ESP_OK)
    return result;
  for (std::size_t i = 0; i < next_count; ++i) {
    if (!validFifoHeader(
            fifo_buffer_[i * kFifoPacket3Size + kFifoHeaderOffset])) {
      // FIFO_DATAは既に消費されておりtimestampとの対応を復元できないため、
      // 再初期化されるまでFIFO readを禁止する。
      fifo_faulted_ = true;
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  for (std::size_t i = 0; i < next_count; ++i)
    (void)parseFifoPacket3(&fifo_buffer_[i * kFifoPacket3Size], data[i]);
  uint64_t next_timestamp_us = fifo_timestamp_us_;
  uint8_t next_remainder = fifo_timestamp_remainder_;
  for (std::size_t i = 0; i < next_count; ++i)
    (void)accumulateTimestamp(data[i].timestamp_ticks, next_timestamp_us,
                              next_remainder);
  fifo_timestamp_us_ = next_timestamp_us;
  fifo_timestamp_remainder_ = next_remainder;
  count = next_count;
  return ESP_OK;
}

esp_err_t ICM42688::readFifo(FifoData *data, std::size_t capacity,
                             std::size_t &count) {
  count = 0;
  if (!initialized_ || spi_ == nullptr || !config_.fifo.enabled)
    return ESP_ERR_INVALID_STATE;
  if (fifo_faulted_)
    return ESP_ERR_INVALID_STATE;
  if (data == nullptr && capacity != 0)
    return ESP_ERR_INVALID_ARG;
  if (capacity == 0) {
    count = 0;
    return ESP_OK;
  }
  std::size_t next_count{};
  esp_err_t result = readFifoBytes(capacity, next_count);
  if (result != ESP_OK)
    return result;
  for (std::size_t i = 0; i < next_count; ++i) {
    if (!validFifoHeader(
            fifo_buffer_[i * kFifoPacket3Size + kFifoHeaderOffset])) {
      // FIFO_DATAは既に消費されておりtimestampとの対応を復元できないため、
      // 再初期化されるまでFIFO readを禁止する。
      fifo_faulted_ = true;
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  uint64_t next_timestamp_us = fifo_timestamp_us_;
  uint8_t next_remainder = fifo_timestamp_remainder_;
  for (std::size_t i = 0; i < next_count; ++i) {
    FifoRawData raw{};
    (void)parseFifoPacket3(&fifo_buffer_[i * kFifoPacket3Size], raw);
    FifoData next{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      next.acceleration_g[axis] =
          raw.acceleration[axis] / accelSensitivity(accel_range_);
      next.angular_velocity_dps[axis] =
          raw.angular_velocity[axis] / gyroSensitivity(gyro_range_);
    }
    next.temperature_celsius = fifoTemperatureCelsius(raw.temperature);
    next.timestamp_ticks = raw.timestamp_ticks;
    next.timestamp_us = accumulateTimestamp(raw.timestamp_ticks,
                                            next_timestamp_us, next_remainder);
    next.acceleration_valid = raw.acceleration_valid;
    next.angular_velocity_valid = raw.angular_velocity_valid;
    next.temperature_valid = raw.temperature_valid;
    next.accel_odr_changed = raw.accel_odr_changed;
    next.gyro_odr_changed = raw.gyro_odr_changed;
    data[i] = next;
  }
  fifo_timestamp_us_ = next_timestamp_us;
  fifo_timestamp_remainder_ = next_remainder;
  count = next_count;
  return ESP_OK;
}

esp_err_t ICM42688::selfTest(SelfTestResult &result, avi::Timeout timeout) {
  if (!initialized_ || spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  if (config_.fifo.enabled)
    return ESP_ERR_NOT_SUPPORTED;
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
