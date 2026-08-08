#include <I2CCREATE.h>
#include <SSCDRRN005PD2A5.h>

extern I2CCREATE i2c;
SSCDRRN005PD2A5 sensor;

esp_err_t readDifferentialPressure(SSCDRRN005PD2A5::Data &data) {
  if (!sensor.initialized()) {
    const esp_err_t result = sensor.begin(i2c);
    if (result != ESP_OK)
      return result;
  }
  return sensor.read(data);
}
