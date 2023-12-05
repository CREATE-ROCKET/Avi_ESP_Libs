/*
 Copyright [2023] <CREATE TokyoTech>
*/
// version: 1.2.0


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

int get = 1; //GetとGet2の切り替えのための変数

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("launched");

    SPIC.begin(VSPI, H3LIS331SCK, H3LIS331MISO, H3LIS331MOSI);
    H3lis331.begin(&SPIC, H3LIS331CS, SPIFREQ);

    uint8_t a;
    a = H3lis331.WhoAmI(); //whoamiを取得
    Serial.print("WhoAmI:");
    Serial.println(a); //取得したwhoamiを表示
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
    if(get == 1){ //H3lis331.Get
        int16_t rx[3];
        H3lis331.Get(rx); //3軸の加速度を取得
        Serial.print(rx[0]); //取得した各軸の加速度を表示
        Serial.print(",");
        Serial.print(rx[1]);
        Serial.print(",");
        Serial.println(rx[2]);
        delay(1000);
    }

    if(get == 2){ //H3lis331.Get2
        int16_t rx[3];
        uint8_t rx_buf[6];
        H3lis331.Get2(rx , rx_buf); //3軸の加速度を取得
        Serial.print(rx[0]); //取得した各軸の加速度を表示
        Serial.print(",");
        Serial.print(rx[1]);
        Serial.print(",");
        Serial.println(rx[2]);
        delay(1000);
    }
}
}