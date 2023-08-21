/*
Copyright [2023] <CREATE TokyoTech>
*/
#include <Arduino.h>
#include <LPS25HB.h>
#include <SPICREATE.h>
#define CS 13
#define SCK 33
#define MISO 25
#define MOSI 26
SPICREATE::SPICreate spi;
LPS lps;
int flag = 0;
void setup() {
    Serial.begin(115200);
    Serial.println("serial set done");
    delay(100);
    /*SPI通信の使用するモジュール、使用するピンの設定*/
    spi.begin(VSPI, SCK, MISO, MOSI);
    Serial.println("spi set done");
    delay(100);
    lps.begin(&spi, CS, 5000000);// LPSのCSピン、SPI通信の設定
    Serial.println("lps set done");
    uint8_t a = lps.WhoAmI();// LPSのwhoamiを取得
    Serial.print("lps whoami is");
    Serial.println(a);
}
void loop() {
    if (Serial.available()) {
        if (Serial.read() == 'l') {
            flag = 1;
        }
    }
    if (flag > 0) {
        uint8_t data[3] = {};
        lps.Get(data);// 生データを三つに分けて取得
        for (int i = 0; i < 3; i++) {
            Serial.print(data[i]);
            Serial.print(",\t");
        }
        Serial.println(lps.Plessure);// 生データから計算して気圧を表示
    }
    delay(1000);
}
