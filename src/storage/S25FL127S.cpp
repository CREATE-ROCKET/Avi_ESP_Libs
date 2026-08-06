#include "S25FL127S.h"

#include <algorithm>
#include "avi_esp_libs/compatibility.h"

namespace {
constexpr uint8_t kRead = 0x03;
constexpr uint8_t kReadJedecId = 0x9F;
constexpr uint8_t kWriteDisable = 0x04;
constexpr uint8_t kWriteEnable = 0x06;
constexpr uint8_t kErase = 0x60;
constexpr uint8_t kProgram = 0x02;
constexpr uint8_t kStatus = 0x05;
constexpr uint8_t kClearStatus = 0x30;
constexpr uint8_t kErrorMask = 0x60;
constexpr uint8_t kProtectionMask = 0x1C;
constexpr uint32_t kMaximumReadFrequencyHz = 50000000;

int64_t deadlineAfter(uint32_t timeout_ms) {
  return avi_micros() + static_cast<int64_t>(timeout_ms) * 1000;
} // 名前なし名前空間
}

S25FL127S::~S25FL127S() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t S25FL127S::begin(SPICREATE &spi, int chip_select,
                            uint32_t frequency_hz) {
  return begin(spi, chip_select, Config{frequency_hz, 1000});
}

esp_err_t S25FL127S::begin(SPICREATE &spi, int chip_select,
                            const Config &config) {
  if (spi_ != nullptr || device_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  if (config.frequency_hz == 0 ||
      config.frequency_hz > kMaximumReadFrequencyHz)
    return ESP_ERR_INVALID_ARG;

  esp_err_t result = spi.addDevice(
      {chip_select, config.frequency_hz, 3, 1}, device_);
  if (result != ESP_OK)
    return result;

  spi_ = &spi;
  ready_timeout_ms_ = config.ready_timeout_ms;

  const auto fail = [this](esp_err_t cause) {
    const esp_err_t cleanup = spi_->removeDevice(device_);
    if (cleanup == ESP_OK)
      spi_ = nullptr;
    return cleanup == ESP_OK ? cause : cleanup;
  };

  result = waitReadyUntil(deadlineAfter(ready_timeout_ms_));
  if (result != ESP_OK)
    return fail(result);

  JedecId id{};
  result = readJedecId(id);
  if (result != ESP_OK)
    return fail(result);
  if (id != kExpectedJedecId)
    return fail(ESP_ERR_INVALID_RESPONSE);

  return ESP_OK;
}

esp_err_t S25FL127S::end() {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = spi_->removeDevice(device_);
  if (result == ESP_OK)
    spi_ = nullptr;
  return result;
}

bool S25FL127S::initialized() const {
  return spi_ != nullptr && device_ != nullptr && spi_->initialized() &&
         spi_->owns(device_);
}

esp_err_t S25FL127S::readJedecId(JedecId &id) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  JedecId value{};
  const esp_err_t result =
      spi_->read(device_, kReadJedecId, value.data(), value.size());
  if (result == ESP_OK)
    id = value;
  return result;
}

esp_err_t S25FL127S::readStatus(uint8_t &status) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, kStatus, status);
}

esp_err_t S25FL127S::waitReadyUntil(int64_t deadline_us) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  for (;;) {
    uint8_t status = 0;
    const esp_err_t result = readStatus(status);
    if (result != ESP_OK)
      return result;
    if ((status & kErrorMask) != 0) {
      (void)spi_->sendCommand(device_, kClearStatus);
      (void)spi_->sendCommand(device_, kWriteDisable);
      return ESP_FAIL;
    }
    if ((status & 0x01U) == 0)
      return ESP_OK;
    if (avi_micros() >= deadline_us)
      return ESP_ERR_TIMEOUT;
    avi_delay_ms(1);
  }
}

esp_err_t S25FL127S::writeEnable() {
  esp_err_t result = spi_->sendCommand(device_, kWriteEnable);
  if (result != ESP_OK)
    return result;
  uint8_t status{};
  result = readStatus(status);
  if (result != ESP_OK)
    return result;
  return (status & 0x02U) != 0 ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t S25FL127S::erase(uint32_t timeout_ms) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  const int64_t deadline_us = deadlineAfter(timeout_ms);
  esp_err_t result = waitReadyUntil(deadline_us);
  if (result != ESP_OK)
    return result;
  uint8_t status{};
  result = readStatus(status);
  if (result != ESP_OK)
    return result;
  if ((status & kProtectionMask) != 0)
    return ESP_ERR_INVALID_STATE;
  if (avi_micros() >= deadline_us)
    return ESP_ERR_TIMEOUT;
  result = writeEnable();
  if (result != ESP_OK)
    return result;
  result = spi_->sendCommand(device_, kErase);
  return result == ESP_OK ? waitReadyUntil(deadline_us) : result;
}

esp_err_t S25FL127S::write(uint32_t address, const uint8_t *data,
                           std::size_t length, uint32_t timeout_ms) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  if (data == nullptr || length == 0 || address >= kCapacity ||
      length > kCapacity - address)
    return ESP_ERR_INVALID_ARG;

  // 全ページで同じ期限を共有し、ページ数に応じてタイムアウトを延長しない。
  const int64_t deadline_us = deadlineAfter(timeout_ms);
  esp_err_t result = waitReadyUntil(deadline_us);
  if (result != ESP_OK)
    return result;

  while (length != 0) {
    if (avi_micros() >= deadline_us)
      return ESP_ERR_TIMEOUT;
    const std::size_t page_remaining = kPageSize - address % kPageSize;
    const std::size_t chunk = std::min(length, page_remaining);

    result = writeEnable();
    if (result != ESP_OK)
      return result;

    spi_transaction_ext_t transaction{};
    transaction.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    transaction.base.cmd = kProgram;
    transaction.base.addr = address;
    transaction.base.length = chunk * 8;
    transaction.base.tx_buffer = data;
    transaction.command_bits = 8;
    transaction.address_bits = 24;
    result = spi_->transmit(device_, transaction.base);
    if (result != ESP_OK)
      return result;
    result = waitReadyUntil(deadline_us);
    if (result != ESP_OK)
      return result;

    address += static_cast<uint32_t>(chunk);
    data += chunk;
    length -= chunk;
  }

  return ESP_OK;
}

esp_err_t S25FL127S::read(uint32_t address, uint8_t *data,
                          std::size_t length) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  if (data == nullptr || length == 0 || address >= kCapacity ||
      length > kCapacity - address)
    return ESP_ERR_INVALID_ARG;

  while (length != 0) {
    const std::size_t chunk = std::min(length, kPageSize);
    spi_transaction_ext_t transaction{};
    transaction.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    transaction.base.cmd = kRead;
    transaction.base.addr = address;
    transaction.base.length = chunk * 8;
    transaction.base.rx_buffer = data;
    transaction.command_bits = 8;
    transaction.address_bits = 24;
    const esp_err_t result = spi_->transmit(device_, transaction.base);
    if (result != ESP_OK)
      return result;

    address += static_cast<uint32_t>(chunk);
    data += chunk;
    length -= chunk;
  }

  return ESP_OK;
}
