#include "ICM20948.h"
#include "spi_sensor.h"
ICM20948::~ICM20948() {
  if (device_)
    (void)end();
}
esp_err_t ICM20948::selectBank(uint8_t bank) {
  return spi_->writeRegister(device_, 0x7F, bank);
}
esp_err_t ICM20948::begin(SPICREATE &s, int cs, uint32_t f) {
  if (device_)
    return ESP_ERR_INVALID_STATE;
  auto r = avi_spi_add(s, cs, f, 0, device_);
  if (r != ESP_OK)
    return r;
  spi_ = &s;
  r = spi_->writeRegister(device_, 0x03, 0x10);
  if (r == ESP_OK)
    r = spi_->writeRegister(device_, 0x06, 0x01);
  if (r == ESP_OK)
    r = selectBank(0x20);
  if (r == ESP_OK)
    r = spi_->writeRegister(device_, 0x14, 0x06);
  if (r == ESP_OK)
    r = spi_->writeRegister(device_, 0x01, 0x06);
  if (r == ESP_OK)
    r = selectBank(0);
  if (r != ESP_OK)
    (void)end();
  return r;
}
esp_err_t ICM20948::end() {
  if (!spi_ || !device_)
    return ESP_ERR_INVALID_STATE;
  auto r = spi_->removeDevice(device_);
  if (r == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
  }
  return r;
}
esp_err_t ICM20948::whoAmI(uint8_t &v) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  auto r = selectBank(0);
  return r == ESP_OK ? spi_->readRegister(device_, 0x80, v) : r;
}
esp_err_t ICM20948::get(Data &d) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  auto r = selectBank(0);
  if (r != ESP_OK)
    return r;
  uint8_t raw[12]{};
  r = avi_spi_read(*spi_, device_, 0xAD, raw, sizeof(raw));
  if (r != ESP_OK)
    return r;
  for (size_t i = 0; i < 6; ++i)
    d.imu[i] = (raw[i * 2] << 8) | raw[i * 2 + 1];
  uint8_t mag[9]{};
  r = avi_spi_read(*spi_, device_, 0xBB, mag, sizeof(mag));
  if (r != ESP_OK)
    return r;
  for (size_t i = 0; i < 3; ++i)
    d.magnetic[i] = (mag[2 + i * 2] << 8) | mag[1 + i * 2];
  return ESP_OK;
}
