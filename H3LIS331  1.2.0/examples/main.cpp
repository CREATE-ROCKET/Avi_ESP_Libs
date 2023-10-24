#include <Arduino.h>
#include <H3LIS331.h>
#include <SPICREATE.h>

#define H3LIS331SCK 33
#define H3LIS331MISO 25
#define H3LIS331MOSI 26
#define H3LIS331CS 32

bool WhoAmI_Ok = false;

#define SPIFREQ 100000

H3LIS331 H3lis331;

SPICREATE::SPICreate SPIC;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("launched");

    SPIC.begin(VSPI, H3LIS331SCK, H3LIS331MISO, H3LIS331MOSI);
    H3lis331.begin(&SPIC, H3LIS331CS, SPIFREQ);

    // WhoAmI
    uint8_t a;
    a = H3lis331.WhoImI();
    Serial.print("WhoAmI:");
    Serial.println(a);
    if (a == 0x32) {
        Serial.println("H3LIS331 is OK");
        WhoAmI_Ok = true;
    } else {
        Serial.println("H3LIS331 is NG");
    }
}

void loop() {
    // put your main code here, to run repeatedly:
    if (WhoAmI_Ok == true) {
        int16_t rx[3];
        H3lis331.Get(rx);
        Serial.print(rx[0]);
        Serial.print(",");
        Serial.print(rx[1]);
        Serial.print(",");
        Serial.println(rx[2]);
        }
}