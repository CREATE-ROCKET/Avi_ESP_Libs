#include <Arduino.h>
#include <ICM20948.h>
#include <SPICREATE.h>

#define ICMSCK 33//18
#define ICMMISO 25//19
#define ICMMOSI 26//23
#define ICMCS 27//33
#define SPIFREQ 1200000

ICM icm20948;

SPICREATE::SPICreate SPIC;


void setup() {
  Serial.begin(115200);
  SPIC.begin(VSPI, ICMSCK, ICMMISO, ICMMOSI);
  icm20948.begin(&SPIC, ICMCS, SPIFREQ);
  uint8_t a;
  a = icm20948.WhoAmI();
  Serial.print(a);

}

void loop() {
  int16_t ICM_data[6];
  icm20948.Get(ICM_data);
  Serial.print(ICM_data[0]);
  Serial.print(",");
  Serial.print(ICM_data[1]);
  Serial.print(",");
  Serial.print(ICM_data[2]);
  Serial.print(",");
  Serial.print(ICM_data[3]);
  Serial.print(",");
  Serial.print(ICM_data[4]);
  Serial.print(",");
  Serial.println(ICM_data[5]);
}