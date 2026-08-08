#include <I2CCREATE.h>
#include <LPS25HB.h>
#include <SSCDRRN005PD2A5.h>

I2CCREATE i2c;
LPS25HB pressure;
SSCDRRN005PD2A5 differential_pressure;

esp_err_t initializeI2cDevices() {
  I2CCREATE::Config config{};
  config.port = I2C_NUM_0;
  config.sda = 8;
  config.scl = 9;
  config.frequency_hz = 400000;
  esp_err_t result = i2c.begin(config);
  if (result == ESP_OK)
    result = pressure.begin(i2c, LPS25HB::Address::high);
  if (result == ESP_OK)
    result = differential_pressure.begin(i2c);
  return result;
}
