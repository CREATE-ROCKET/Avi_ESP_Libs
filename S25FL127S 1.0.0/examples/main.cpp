/*
Copyright [2023] <CREATE TokyoTech>
*/

#include <Arduino.h>
#include <SPIflash.h>

#define SPIFREQ 5000000

uint8_t tx[256] = {};
uint8_t rx[256] = {};

#define flashCS 27
#define SCK1 33
#define MISO1 25
#define MOSI1 26

SPICREATE::SPICreate SPIC1;
Flash flash1;

// SPIflash S25FL127S用サンプルコード

void setup() {
    // FlashのCSピンを非選択に設定
    digitalWrite(flashCS, HIGH);
    // シリアルの初期化
    Serial.begin(115200);
    // SPIの初期化
    SPIC1.begin(VSPI, SCK1, MISO1, MOSI1);
    // Flashの初期化
    flash1.begin(&SPIC1, flashCS, SPIFREQ);
    // Flashのデータを一括消去
    flash1.erase();
}

int count = 0;

void loop() {
    if (count < 3) {
        // 書き込むデータを作る
        for (int i = 0; i < 256; i++) {
            tx[i] = i;
        }
        // 書き込み
        flash1.write(count * 0x100, tx);
        delay(1000);
        // 読み込み
        flash1.read(count * 0x100, rx);
        // 読み込んだデータをシリアルで表示
        Serial.print("at page ");
        Serial.println(count + 1);
        delay(100);
        for (int i = 0; i < 256; i++) {
            Serial.print(rx[i]);
            Serial.print(" ");
        }
        count++;
        Serial.println("");
        delay(100);
    }
}
