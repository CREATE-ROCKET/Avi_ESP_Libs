/*
Copyright [2023] <CREATE TokyoTech>
*/

#include <Arduino.h>
#include <S25FL512S.h>

uint8_t tx[256] = {};
uint8_t rx[256] = {};

#define flashCS 27
#define SCK1 33
#define MISO1 25
#define MOSI1 26
#define SPIFREQ SPI_MASTER_FREQ_8M

SPICREATE::SPICreate SPIC1;
Flash flash1;

const int PageSize = 0x100;

// SPIflash S25FL512S用サンプルコード

void setup() {
    // FlashのCSピンを非選択に設定
    digitalWrite(flashCS, HIGH);
    // シリアルの初期化
    Serial.begin(115200);
    // SPIの初期化
    SPIC1.begin(VSPI, SCK1, MISO1, MOSI1);
    // Flashの初期化
    flash1.begin(&SPIC1, flashCS, SPIFREQ);

    // Flashのデータを一括消去開始
    flash1.erase_silent();

    while (flash1.erase_status() == erase_status_code_define::deleting) {
        // ここに消去中の処理を書く
        Serial.println("Deleting Now!");
        Serial.println(flash1.erase_status());
        delay(100);  // ここは何秒でもいい
    }
}

int count = 0;

void loop() {
    if (count < 3) {
        // 書き込むデータを作る
        for (int i = 0; i < 256; i++) {
            tx[i] = i;
        }
        // 書き込み
        flash1.write(count * PageSize, tx);
        delay(1000);
        // 読み込み
        flash1.read(count * PageSize, rx);
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
