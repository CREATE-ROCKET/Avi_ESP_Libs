#include "ICM20602.h"
ICM20602::~ICM20602() {
  if (device_)
    (void)end();
}
esp_err_t ICM20602::begin(SPICREATE &spi, int cs, uint32_t f) {
  if (device_)
    return ESP_ERR_INVALID_STATE;
  esp_err_t r = spi.addDevice({cs, f, 0, 1}, device_);
  if (r != ESP_OK)
    return r;
  spi_ = &spi;
  r = spi_->writeRegister(device_, 0x6B, 0x01);
  if (r != ESP_OK)
    (void)end();
  return r;
}
esp_err_t ICM20602::end() {
  if (!spi_ || !device_)
    return ESP_ERR_INVALID_STATE;
  auto r = spi_->removeDevice(device_);
  if (r == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
  }
  return r;
}
esp_err_t ICM20602::whoAmI(uint8_t &v) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, 0xF5, v);
}
esp_err_t ICM20602::get(Data &d) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw[14]{};
  auto r = spi_->read(device_, 0xBB, raw, sizeof(raw));
  if (r != ESP_OK)
    return r;
  for (size_t i = 0; i < 3; ++i)
    d[i] = (raw[i * 2] << 8) | raw[i * 2 + 1];
  for (size_t i = 0; i < 3; ++i)
    d[i + 3] = (raw[8 + i * 2] << 8) | raw[9 + i * 2];
  return ESP_OK;
}
