/*
Copyright [2023] <CREATE TokyoTech>
*/

#include <Arduino.h>
#include <ICM20948.h>
#include <SPICREATE.h>

#define ICMSCK 33
#define ICMMISO 25
#define ICMMOSI 26
#define ICMCS 27
#define SPIFREQ 1200000

ICM icm20948;

SPICREATE::SPICreate SPIC;

int16_t ICM_data[6];
uint8_t ICM_data_raw[14];

void setup() {
  delay(3000);
    Serial.begin(115200);
    SPIC.begin(VSPI, ICMSCK, ICMMISO, ICMMOSI);
    icm20948.begin(&SPIC, ICMCS, SPIFREQ);
    uint8_t a;
    a = icm20948.WhoAmI();
    Serial.print(a);
}

void loop() {
    
    icm20948.Get(ICM_data, ICM_data_raw);
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
    delay(1000);
}
