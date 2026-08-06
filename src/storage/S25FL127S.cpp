#include "S25FL127S.h"
#include "avi_esp_libs/compatibility.h"
namespace {
constexpr uint8_t kRead = 0x03, kWriteEnable = 0x06, kErase = 0x60,
                  kProgram = 0x02, kStatus = 0x05;
constexpr uint32_t kCapacity = 16U * 1024U * 1024U;
} // namespace
S25FL127S::~S25FL127S() {
  if (device_)
    (void)end();
}
esp_err_t S25FL127S::begin(SPICREATE &s, int cs, uint32_t f) {
  if (device_)
    return ESP_ERR_INVALID_STATE;
  auto r = s.addDevice({cs, f, 3, 1}, device_);
  if (r == ESP_OK)
    spi_ = &s;
  return r;
}
esp_err_t S25FL127S::end() {
  if (!spi_ || !device_)
    return ESP_ERR_INVALID_STATE;
  auto r = spi_->removeDevice(device_);
  if (r == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
  }
  return r;
}
esp_err_t S25FL127S::waitReady(uint32_t timeout) {
  const int64_t deadline = avi_micros() + int64_t(timeout) * 1000;
  do {
    uint8_t s = 0;
    auto r = spi_->readRegister(device_, kStatus, s);
    if (r != ESP_OK)
      return r;
    if ((s & 1) == 0)
      return ESP_OK;
    avi_delay_ms(1);
  } while (avi_micros() < deadline);
  return ESP_ERR_TIMEOUT;
}
esp_err_t S25FL127S::erase(uint32_t t) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  auto r = spi_->sendCommand(device_, kWriteEnable);
  if (r == ESP_OK)
    r = spi_->sendCommand(device_, kErase);
  return r == ESP_OK ? waitReady(t) : r;
}
esp_err_t S25FL127S::write(uint32_t a, const uint8_t *d, size_t n, uint32_t t) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  if (!d || n == 0 || n > kPageSize || a >= kCapacity || n > kCapacity - a ||
      a / kPageSize != (a + n - 1) / kPageSize)
    return ESP_ERR_INVALID_ARG;
  auto r = spi_->sendCommand(device_, kWriteEnable);
  if (r != ESP_OK)
    return r;
  spi_transaction_ext_t x{};
  x.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
  x.base.cmd = kProgram;
  x.base.addr = a;
  x.base.length = n * 8;
  x.base.tx_buffer = d;
  x.command_bits = 8;
  x.address_bits = 24;
  r = spi_->transmit(device_, x.base);
  return r == ESP_OK ? waitReady(t) : r;
}
esp_err_t S25FL127S::read(uint32_t a, uint8_t *d, size_t n) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  if (!d || n == 0 || a >= kCapacity || n > kCapacity - a)
    return ESP_ERR_INVALID_ARG;
  spi_transaction_ext_t x{};
  x.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
  x.base.cmd = kRead;
  x.base.addr = a;
  x.base.length = n * 8;
  x.base.rx_buffer = d;
  x.command_bits = 8;
  x.address_bits = 24;
  return spi_->transmit(device_, x.base);
}
