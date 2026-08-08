#include <AS5047D.h>
#include <SPICREATE.h>

SPICREATE spi;
AS5047D encoder;

void setup() {
  if (spi.begin(SPI2_HOST, 12, 13, 11) != ESP_OK)
    return;
  if (encoder.begin(spi, 10) != ESP_OK)
    (void)spi.end();
}

void loop() {
  AS5047D::Data data{};
  if (encoder.read(data) != ESP_OK)
    return;
  // angle_raw、angle_degrees、angle_radiansを利用する。
}
