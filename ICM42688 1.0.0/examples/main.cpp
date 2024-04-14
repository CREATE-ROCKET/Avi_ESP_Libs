#include <Arduino.h>
#include "ICM42688.h"

namespace ICMPIN
{
    const int SCK = 14;
    const int MISO = 12;
    const int MOSI = 13;
    const int CS = 15;
}

ICM icm42688;

SPICREATE::SPICreate SPIC;

void setup()
{
    Serial.begin(115200);
    SPIC.begin(VSPI, ICMPIN::SCK, ICMPIN::MISO, ICMPIN::MOSI);
    icm42688.begin(&SPIC, ICMPIN::CS, 1000000);
    Serial.print("ICM WhoAmI checking ,");
    while (icm42688.WhoAmI() != 0x47)
    {
        delay(100);
        Serial.print(",");
    }
    Serial.println("ICM Connected!");
}

void loop()
{
    int16_t ICM_data[7];
    icm42688.Get(ICM_data);
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