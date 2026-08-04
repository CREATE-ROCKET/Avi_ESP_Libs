#include "H3LIS331.h"
#include "spi_sensor.h"
H3LIS331::~H3LIS331() {
  if (device_)
    (void)end();
}
esp_err_t H3LIS331::begin(SPICREATE &s, int cs, uint32_t f) {
  if (device_)
    return ESP_ERR_INVALID_STATE;
  auto r = avi_spi_add(s, cs, f, 0, device_);
  if (r != ESP_OK)
    return r;
  spi_ = &s;
  const uint8_t reg[][2] = {
      {0x20, 0x3F}, {0x21, 0}, {0x22, 0}, {0x23, 0x30}, {0x24, 0}};
  for (auto &p : reg) {
    r = spi_->writeRegister(device_, p[0], p[1]);
    if (r != ESP_OK) {
      (void)end();
      return r;
    }
  }
  return ESP_OK;
}
esp_err_t H3LIS331::end() {
  if (!spi_ || !device_)
    return ESP_ERR_INVALID_STATE;
  auto r = spi_->removeDevice(device_);
  if (r == ESP_OK) {
    spi_ = nullptr;
    device_ = nullptr;
  }
  return r;
}
esp_err_t H3LIS331::whoAmI(uint8_t &v) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, 0x8F, v);
}
esp_err_t H3LIS331::get(Data &d) {
  if (!spi_)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw[6]{};
  auto r = avi_spi_read(*spi_, device_, 0xE8, raw, sizeof(raw));
  if (r != ESP_OK)
    return r;
  for (size_t i = 0; i < 3; ++i)
    d[i] = (raw[i * 2 + 1] << 8) | raw[i * 2];
  return ESP_OK;
}
