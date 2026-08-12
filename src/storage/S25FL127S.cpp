#include "S25FL127S.h"

#include "../compatibility/timeout_internal.h"
#include "avi_esp_libs/compatibility.h"
#include <algorithm>

namespace {
constexpr uint8_t kRead = 0x03;
constexpr uint8_t kReadJedecId = 0x9F;
constexpr uint8_t kWriteDisable = 0x04;
constexpr uint8_t kWriteEnable = 0x06;
constexpr uint8_t kErase = 0x60;
constexpr uint8_t kEraseBlock = 0xD8;
constexpr uint8_t kProgram = 0x02;
constexpr uint8_t kStatus = 0x05;
constexpr uint8_t kStatus2 = 0x07;
constexpr uint8_t kClearStatus = 0x30;
constexpr uint8_t kErrorMask = 0x60;
constexpr uint8_t kProtectionMask = 0x1C;
constexpr uint32_t kMaximumReadFrequencyHz = 50000000;

esp_err_t finiteDeadline(avi::Timeout timeout, int64_t &deadline_us) {
  if (!timeout.isFinite())
    return ESP_ERR_INVALID_ARG;
  avi::internal::Deadline deadline{};
  const esp_err_t result = avi::internal::makeDeadline(timeout, deadline);
  if (result == ESP_OK)
    deadline_us = deadline.microseconds;
  return result;
}
} // namespace

S25FL127S::~S25FL127S() {
  if (device_ != nullptr)
    (void)end();
}

esp_err_t S25FL127S::begin(SPICREATE &spi, int chip_select,
                           uint32_t frequency_hz) {
  return begin(spi, chip_select,
               Config{frequency_hz, avi::Timeout::milliseconds(1000)});
}

esp_err_t S25FL127S::begin(SPICREATE &spi, int chip_select,
                           const Config &config) {
  if (spi_ != nullptr || device_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  int64_t deadline_us{};
  if (config.frequency_hz == 0 ||
      config.frequency_hz > kMaximumReadFrequencyHz ||
      finiteDeadline(config.ready_timeout, deadline_us) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  esp_err_t result =
      spi.addDevice({chip_select, config.frequency_hz, 3, 1}, device_);
  if (result != ESP_OK)
    return result;

  spi_ = &spi;
  const auto fail = [this](esp_err_t cause) {
    const esp_err_t cleanup = spi_->removeDevice(device_);
    if (cleanup == ESP_OK)
      spi_ = nullptr;
    return cleanup == ESP_OK ? cause : cleanup;
  };

  result = waitReadyUntil(deadline_us);
  if (result != ESP_OK)
    return fail(result);

  JedecId id{};
  result = readJedecId(id);
  if (result != ESP_OK)
    return fail(result);
  if (id != kExpectedJedecId)
    return fail(ESP_ERR_INVALID_RESPONSE);

  uint8_t status2{};
  result = spi_->readRegister(device_, kStatus2, status2);
  if (result != ESP_OK)
    return fail(result);
  // SR2[7] (D8h_O): 0は64 KiB、1は256 KiBのD8h消去単位。
  block_size_ = (status2 & 0x80U) != 0 ? 256U * 1024U : 64U * 1024U;

  return ESP_OK;
}

esp_err_t S25FL127S::end() {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  const esp_err_t result = spi_->removeDevice(device_);
  if (result == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
    block_size_ = 0;
  }
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

esp_err_t S25FL127S::getStatus(Status &status) {
  uint8_t raw{};
  const esp_err_t result = readStatus(raw);
  if (result == ESP_OK) {
    Status next{};
    next.busy = (raw & 0x01U) != 0;
    next.write_enable = (raw & 0x02U) != 0;
    next.protected_area = (raw & kProtectionMask) != 0;
    next.erase_error = (raw & 0x20U) != 0;
    next.program_error = (raw & 0x40U) != 0;
    status = next;
  }
  return result;
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

esp_err_t S25FL127S::rangeErased(uint32_t address, std::size_t length,
                                 bool &erased) {
  erased = false;
  if (!initialized() || length == 0 || address >= kCapacity ||
      length > kCapacity - address)
    return ESP_ERR_INVALID_ARG;

  std::array<uint8_t, kPageSize> buffer{};
  while (length != 0) {
    const std::size_t chunk = std::min(length, buffer.size());
    const esp_err_t result = read(address, buffer.data(), chunk);
    if (result != ESP_OK)
      return result;
    if (!std::all_of(buffer.begin(), buffer.begin() + chunk,
                     [](uint8_t value) { return value == 0xFF; }))
      return ESP_OK;
    address += static_cast<uint32_t>(chunk);
    length -= chunk;
  }
  erased = true;
  return ESP_OK;
}

esp_err_t S25FL127S::blankCheckUntil(int64_t deadline_us) {
  uint32_t address = 0;
  while (address < kCapacity) {
    if (avi_micros() >= deadline_us)
      return ESP_ERR_TIMEOUT;
    const std::size_t chunk =
        std::min<std::size_t>(kPageSize, kCapacity - address);
    bool erased{};
    const esp_err_t result = rangeErased(address, chunk, erased);
    if (result != ESP_OK)
      return result;
    if (!erased)
      return ESP_ERR_INVALID_RESPONSE;
    address += static_cast<uint32_t>(chunk);
  }
  return ESP_OK;
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

esp_err_t S25FL127S::eraseChip(avi::Timeout timeout) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  int64_t deadline_us{};
  if (finiteDeadline(timeout, deadline_us) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
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
  if (result != ESP_OK)
    return result;
  result = waitReadyUntil(deadline_us);
  return result == ESP_OK ? blankCheckUntil(deadline_us) : result;
}

esp_err_t S25FL127S::eraseAddressed(uint8_t command, uint32_t address,
                                    std::size_t alignment,
                                    avi::Timeout timeout) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  if (address >= kCapacity || address % alignment != 0)
    return ESP_ERR_INVALID_ARG;
  int64_t deadline_us{};
  if (finiteDeadline(timeout, deadline_us) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  esp_err_t result = waitReadyUntil(deadline_us);
  if (result != ESP_OK)
    return result;
  Status status{};
  result = getStatus(status);
  if (result != ESP_OK)
    return result;
  if (status.protected_area)
    return ESP_ERR_INVALID_STATE;
  result = writeEnable();
  if (result != ESP_OK)
    return result;
  spi_transaction_ext_t transaction{};
  transaction.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
  transaction.base.cmd = command;
  transaction.base.addr = address;
  transaction.command_bits = 8;
  transaction.address_bits = 24;
  result = spi_->transmit(device_, transaction.base);
  return result == ESP_OK ? waitReadyUntil(deadline_us) : result;
}

esp_err_t S25FL127S::eraseBlock(uint32_t address, avi::Timeout timeout) {
  if (block_size_ == 0)
    return ESP_ERR_INVALID_STATE;
  return eraseAddressed(kEraseBlock, address, block_size_, timeout);
}

esp_err_t S25FL127S::write(uint32_t address, const uint8_t *data,
                           std::size_t length, avi::Timeout timeout) {
  if (!initialized())
    return ESP_ERR_INVALID_STATE;
  if (data == nullptr || length == 0 || address >= kCapacity ||
      length > kCapacity - address)
    return ESP_ERR_INVALID_ARG;

  // Automatic ECC unitへ2回programしない。最初のpartial programは許容するが、
  // 触れる16-byte unitはすべてwrite前にerase状態でなければならない。
  const uint32_t ecc_begin =
      address - static_cast<uint32_t>(address % kEccUnitSize);
  const uint32_t write_end = address + static_cast<uint32_t>(length);
  const uint32_t ecc_end = static_cast<uint32_t>(
      (static_cast<uint64_t>(write_end) + kEccUnitSize - 1U) /
      kEccUnitSize * kEccUnitSize);
  bool erased{};
  esp_err_t result = rangeErased(ecc_begin, ecc_end - ecc_begin, erased);
  if (result != ESP_OK)
    return result;
  if (!erased)
    return ESP_ERR_INVALID_STATE;

  // 全ページで同じ期限を共有し、ページ数に応じてタイムアウトを延長しない。
  int64_t deadline_us{};
  if (finiteDeadline(timeout, deadline_us) != ESP_OK)
    return ESP_ERR_INVALID_ARG;
  result = waitReadyUntil(deadline_us);
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

esp_err_t S25FL127S::read(uint32_t address, uint8_t *data, std::size_t length) {
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

esp_err_t S25FL127S::readByte(uint32_t address, uint8_t &value) {
  uint8_t next{};
  const esp_err_t result = read(address, &next, 1);
  if (result == ESP_OK)
    value = next;
  return result;
}

esp_err_t S25FL127S::writeByte(uint32_t address, uint8_t value,
                               avi::Timeout timeout) {
  return write(address, &value, 1, timeout);
}
