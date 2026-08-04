#include "ICM42688.h"
#include "spi_sensor.h"

ICM42688::~ICM42688() {
  if (device_ != nullptr)
    (void)end();
}
esp_err_t ICM42688::begin(SPICREATE &spi, int cs, uint32_t frequency) {
  if (device_ != nullptr)
    return ESP_ERR_INVALID_STATE;
  esp_err_t result = avi_spi_add(spi, cs, frequency, 0, device_);
  if (result != ESP_OK)
    return result;
  spi_ = &spi;
  result = spi_->writeRegister(device_, 0x4E, 0x0F);
  if (result != ESP_OK) {
    (void)end();
  }
  return result;
}
esp_err_t ICM42688::end() {
  if (spi_ == nullptr || device_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  const esp_err_t r = spi_->removeDevice(device_);
  if (r == ESP_OK) {
    device_ = nullptr;
    spi_ = nullptr;
  }
  return r;
}
esp_err_t ICM42688::whoAmI(uint8_t &value) {
  if (spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  return spi_->readRegister(device_, 0xF5, value);
}
esp_err_t ICM42688::get(Data &data) {
  if (spi_ == nullptr)
    return ESP_ERR_INVALID_STATE;
  uint8_t raw[12]{};
  const esp_err_t r = avi_spi_read(*spi_, device_, 0x9F, raw, sizeof(raw));
  if (r != ESP_OK)
    return r;
  for (size_t i = 0; i < data.size(); ++i)
    data[i] = static_cast<int16_t>((raw[i * 2] << 8) | raw[i * 2 + 1]);
  return ESP_OK;
}
