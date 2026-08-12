#include "AS5047D.h"

#include "avi_esp_libs/compatibility.h"

namespace {
constexpr uint16_t kNop = 0x0000;
constexpr uint16_t kErrorFlags = 0x0001;
constexpr uint16_t kDiagnostics = 0x3FFC;
constexpr uint16_t kMagnitude = 0x3FFD;
constexpr uint16_t kZeroPositionLow = 0x0017;
constexpr uint16_t kAngleUncompensated = 0x3FFE;
constexpr uint16_t kAngleCompensated = 0x3FFF;
constexpr uint16_t kRead = 0x4000;
constexpr uint16_t kError = 0x4000;
constexpr uint16_t kDataMask = 0x3FFF;
constexpr uint32_t kMaximumFrequencyHz = 10000000;
constexpr uint32_t kChipSelectSetupNanoseconds = 350;
constexpr float kDegreesPerCount = 360.0F / 16384.0F;
constexpr float kRadiansPerCount = 6.28318530717958647692F / 16384.0F;

void waitChipSelectHigh() {
  // esp_timer_get_time()の1 us分解能を使い、2 usだけbusy-waitする。
  // 350 ns要件に余裕があり、各frame間だけの有限待機なのでtaskを長時間塞がない。
  const int64_t started_at = avi_micros();
  while (avi_micros() - started_at < 2) {
  }
}

constexpr bool hasOddParity(uint16_t value) {
  bool odd = false;
  while (value != 0) {
    odd = !odd;
    value = static_cast<uint16_t>(value & static_cast<uint16_t>(value - 1));
  }
  return odd;
}
constexpr uint16_t withEvenParity(uint16_t value) {
  return hasOddParity(value) ? static_cast<uint16_t>(value | 0x8000) : value;
}
constexpr uint16_t makeReadCommand(uint16_t address) {
  return withEvenParity(static_cast<uint16_t>(kRead | (address & kDataMask)));
}
constexpr uint16_t angleFromResponse(uint16_t response) {
  return static_cast<uint16_t>(response & kDataMask);
}
constexpr AS5047D::ErrorFlags decodeErrors(uint16_t value) {
  return {(value & 0x04) != 0, (value & 0x02) != 0, (value & 0x01) != 0};
}
constexpr uint8_t chipSelectSetupCycles(uint32_t frequency_hz) {
  return static_cast<uint8_t>(
      (static_cast<uint64_t>(frequency_hz) * kChipSelectSetupNanoseconds +
       999999999ULL) /
      1000000000ULL);
}
constexpr bool isSensorDiagnosticFault(uint16_t diagnostics,
                                       uint16_t zero_position) {
  const bool magnetic_high_contributes =
      (zero_position & 0x0040) != 0 && (diagnostics & 0x0400) != 0;
  const bool magnetic_low_contributes =
      (zero_position & 0x0080) != 0 && (diagnostics & 0x0800) != 0;
  return magnetic_high_contributes || magnetic_low_contributes ||
         (diagnostics & 0x0200) != 0 || (diagnostics & 0x0100) == 0;
}
static_assert(makeReadCommand(kAngleCompensated) == 0xFFFF);
static_assert(makeReadCommand(kNop) == 0xC000);
static_assert(makeReadCommand(kErrorFlags) == 0x4001);
static_assert(chipSelectSetupCycles(8000000) == 3);
static_assert(chipSelectSetupCycles(10000000) == 4);
static_assert(!isSensorDiagnosticFault(0x0100, 0x0000));
static_assert(isSensorDiagnosticFault(0x0400, 0x0040));
static_assert(isSensorDiagnosticFault(0x0800, 0x0080));
static_assert(isSensorDiagnosticFault(0x0200, 0x0000));
static_assert(!hasOddParity(makeReadCommand(kAngleUncompensated)));
static_assert(angleFromResponse(0xFFFF) == 0x3FFF);
static_assert(decodeErrors(0x07).framing_error);
static_assert(16383.0F * kDegreesPerCount < 360.0F);
} // namespace

AS5047D::~AS5047D() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t AS5047D::begin(SPICREATE &spi, int chip_select) {
  return begin(spi, chip_select, Config{});
}

esp_err_t AS5047D::begin(SPICREATE &spi, int chip_select,
                         const Config &config) {
  if (initialized_ || (spi_ == nullptr) != (device_ == nullptr))
    return ESP_ERR_INVALID_STATE;
  if (device_ != nullptr) {
    // 前回のcleanupだけが失敗していた場合は、次のbeginで安全に再試行する。
    const esp_err_t cleanup = spi_->removeDevice(device_);
    if (cleanup != ESP_OK)
      return cleanup;
    spi_ = nullptr;
  }
  if (config.frequency_hz == 0 || config.frequency_hz > kMaximumFrequencyHz ||
      static_cast<uint8_t>(config.angle_source) > 1)
    return ESP_ERR_INVALID_ARG;

  // CSn fallingからfirst clockまでdatasheet要求の350 ns以上を確保する。
  const uint8_t cs_setup_cycles =
      chipSelectSetupCycles(config.frequency_hz);
  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 1, 1, 1,
                     cs_setup_cycles},
                    device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;

  // 電源投入直後でも最初の有効角度が得られるまで有限時間待つ。
  avi_delay_ms(10);

  // 前回reset以前のSPI errorを角度responseへ誤帰属させない。
  ErrorFlags startup_errors{};
  result = readErrorFlagsInternal(startup_errors);
  uint16_t angle{};
  if (result == ESP_OK)
    result = readRegister(config.angle_source == AngleSource::compensated
                            ? kAngleCompensated
                            : kAngleUncompensated,
                        angle);
  if (result != ESP_OK) {
    const esp_err_t cleanup = spi_->removeDevice(device_);
    if (cleanup == ESP_OK)
      spi_ = nullptr;
    return cleanup == ESP_OK ? result : cleanup;
  }
  config_ = config;
  last_error_flags_ = startup_errors;
  pipeline_active_ = false;
  initialized_ = true;
  return ESP_OK;
}

esp_err_t AS5047D::end() {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  initialized_ = false;
  pipeline_active_ = false;
  const esp_err_t result = spi_->removeDevice(device_);
  if (result != ESP_OK)
    return result;
  spi_ = nullptr;
  config_ = {};
  last_error_flags_ = {};
  return ESP_OK;
}

esp_err_t AS5047D::transferFrame(uint16_t tx, uint16_t &rx) {
  spi_transaction_t transaction{};
  transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  transaction.length = 16;
  transaction.tx_data[0] = static_cast<uint8_t>(tx >> 8);
  transaction.tx_data[1] = static_cast<uint8_t>(tx);
  const esp_err_t result = spi_->pollingTransmit(device_, transaction);
  waitChipSelectHigh();
  if (result != ESP_OK)
    return result;
  const uint16_t next =
      static_cast<uint16_t>(static_cast<uint16_t>(transaction.rx_data[0]) << 8 |
                            transaction.rx_data[1]);
  if (hasOddParity(next))
    return ESP_ERR_INVALID_CRC;
  rx = next;
  return ESP_OK;
}

uint16_t AS5047D::angleReadCommand() const {
  return makeReadCommand(config_.angle_source == AngleSource::compensated
                             ? kAngleCompensated
                             : kAngleUncompensated);
}

esp_err_t AS5047D::readRegister(uint16_t address, uint16_t &value) {
  uint16_t response{};
  const esp_err_t result = readRegisterResponse(address, response);
  if (result != ESP_OK)
    return result;
  if ((response & kError) != 0)
    return handleErrorFlag();
  value = angleFromResponse(response);
  return ESP_OK;
}

esp_err_t AS5047D::readRegisterResponse(uint16_t address,
                                        uint16_t &response) {
  uint16_t previous_response{};
  // 同じframeのMISOは1つ前のcommandへのresponseなので、parityだけ検証する。
  esp_err_t result = transferFrame(makeReadCommand(address), previous_response);
  if (result != ESP_OK)
    return result;
  // NOP readを送り、直前に送ったrequested registerのresponseを回収する。
  return transferFrame(makeReadCommand(kNop), response);
}

esp_err_t AS5047D::readErrorFlagsInternal(ErrorFlags &flags) {
  uint16_t response{};
  const esp_err_t result = readRegisterResponse(kErrorFlags, response);
  if (result != ESP_OK)
    return result;
  flags = decodeErrors(angleFromResponse(response));
  return ESP_OK;
}

esp_err_t AS5047D::handleErrorFlag() {
  ErrorFlags flags{};
  const esp_err_t result = readErrorFlagsInternal(flags);
  if (result != ESP_OK)
    return result;
  last_error_flags_ = flags;
  if (flags.parity_error)
    return ESP_ERR_INVALID_CRC;
  if (flags.invalid_command || flags.framing_error)
    return ESP_ERR_INVALID_RESPONSE;

  // ERRFLに通信errorがないEFはsensor diagnostic由来かを確認する。
  uint16_t diagnostics_response{};
  esp_err_t diagnostic_result =
      readRegisterResponse(kDiagnostics, diagnostics_response);
  if (diagnostic_result != ESP_OK)
    return diagnostic_result;
  uint16_t zero_position_response{};
  diagnostic_result =
      readRegisterResponse(kZeroPositionLow, zero_position_response);
  if (diagnostic_result != ESP_OK)
    return diagnostic_result;

  const uint16_t diagnostics = angleFromResponse(diagnostics_response);
  const uint16_t zero_position = angleFromResponse(zero_position_response);
  return isSensorDiagnosticFault(diagnostics, zero_position)
             ? ESP_ERR_INVALID_STATE
             : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t AS5047D::readAndClearErrorFlags(ErrorFlags &flags) {
  if (!initialized_ || pipeline_active_)
    return ESP_ERR_INVALID_STATE;
  ErrorFlags next{};
  const esp_err_t result = readErrorFlagsInternal(next);
  if (result == ESP_OK) {
    last_error_flags_ = next;
    flags = next;
  }
  return result;
}

esp_err_t AS5047D::readRaw(RawData &data) {
  if (!initialized_ || pipeline_active_)
    return ESP_ERR_INVALID_STATE;
  uint16_t angle{};
  const esp_err_t result = readRegister(
      config_.angle_source == AngleSource::compensated ? kAngleCompensated
                                                       : kAngleUncompensated,
      angle);
  if (result == ESP_OK)
    data = {angle};
  return result;
}

esp_err_t AS5047D::read(Data &data) {
  RawData raw{};
  const esp_err_t result = readRaw(raw);
  if (result != ESP_OK)
    return result;
  data = {raw.angle, raw.angle * kDegreesPerCount,
          raw.angle * kRadiansPerCount};
  return ESP_OK;
}

esp_err_t AS5047D::startPipelinedRead() {
  if (!initialized_ || pipeline_active_)
    return ESP_ERR_INVALID_STATE;

  uint16_t previous_response{};
  const esp_err_t result = transferFrame(angleReadCommand(), previous_response);
  if (result != ESP_OK)
    return result;

  // prime frameのMISOは以前のcommandへのresponseであり、ANGLEのEFではない。
  pipeline_active_ = true;
  return ESP_OK;
}

esp_err_t AS5047D::readPipelinedRaw(RawData &data) {
  if (!initialized_ || !pipeline_active_)
    return ESP_ERR_INVALID_STATE;

  uint16_t response{};
  const esp_err_t result = transferFrame(angleReadCommand(), response);
  if (result != ESP_OK) {
    pipeline_active_ = false;
    return result;
  }
  if ((response & kError) != 0) {
    pipeline_active_ = false;
    return handleErrorFlag();
  }

  data = {angleFromResponse(response)};
  return ESP_OK;
}

esp_err_t AS5047D::readPipelined(Data &data) {
  RawData raw{};
  const esp_err_t result = readPipelinedRaw(raw);
  if (result != ESP_OK)
    return result;
  data = {raw.angle, raw.angle * kDegreesPerCount,
          raw.angle * kRadiansPerCount};
  return ESP_OK;
}

esp_err_t AS5047D::stopPipelinedRead() {
  if (!initialized_ || !pipeline_active_)
    return ESP_ERR_INVALID_STATE;
  pipeline_active_ = false;
  return ESP_OK;
}

esp_err_t AS5047D::getStatus(Status &status) {
  if (!initialized_ || pipeline_active_)
    return ESP_ERR_INVALID_STATE;
  uint16_t diagnostics{};
  esp_err_t result = readRegister(kDiagnostics, diagnostics);
  if (result != ESP_OK)
    return result;
  uint16_t magnitude{};
  result = readRegister(kMagnitude, magnitude);
  if (result != ESP_OK)
    return result;
  status = {(diagnostics & 0x0800) != 0,       (diagnostics & 0x0400) != 0,
            (diagnostics & 0x0200) != 0,       (diagnostics & 0x0100) != 0,
            static_cast<uint8_t>(diagnostics), magnitude};
  return ESP_OK;
}
