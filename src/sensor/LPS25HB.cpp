#include "LPS25HB.h"
LPS25HB::~LPS25HB() {
  if (device_)
    (void)end();
}
esp_err_t LPS25HB::begin(SPICREATE &s, int cs, uint32_t f) {
  if (device_)
    return ESP_ERR_INVALID_STATE;
  auto r = s.addDevice({cs, f, 3, 1}, device_);
  if (r != ESP_OK)
    return r;
  spi_ = &s;
  r = spi_->writeRegister(device_, 0x21, 0x08);
  if (r == ESP_OK)
    r = spi_->writeRegister(device_, 0x20, 0xC0);
  if (r != ESP_OK)
    (void)end();
  return r;
}
esp_err_t LPS25HB::end() {
  if (!spi_ || !device_)
    return ESP_ERR_INVALID_STATE;
  auto r = spi_->removeDevice(device_);
  if (r == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
  }
  return r;
}
esp_err_t LPS25HB::whoAmI(uint8_t &v) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, 0x8F, v);
}
esp_err_t LPS25HB::get(Data &d) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw[3]{};
  for (uint8_t i = 0; i < 3; ++i) {
    auto r =
        spi_->readRegister(device_, static_cast<uint8_t>(0xA8 + i), raw[i]);
    if (r != ESP_OK)
      return r;
  }
  d.raw = (uint32_t(raw[2]) << 16) | (uint32_t(raw[1]) << 8) | raw[0];
  d.pascals = static_cast<int32_t>((d.raw * 100U) / 4096U);
  return ESP_OK;
}
