#include <Arduino.h>
#include <S25FL512S.h>

#define SPIFREQ 5000000

uint8_t tx[256] = {};
uint8_t rx[256] = {};

#define flashCS 27
#define SCK1 33
#define MISO1 25
#define MOSI1 26

SPICREATE::SPICreate SPIC;
Flash flash;

void setup() {
    // FlashのCSピンを非選択に設定
    digitalWrite(flashCS, HIGH);
    // シリアルの初期化
    Serial.begin(115200);
    // SPIの初期化
    SPIC.begin(VSPI, SCK1, MISO1, MOSI1);
    // Flashの初期化
    flash.begin(&SPIC, flashCS, SPIFREQ);

    Serial.println("Start");
    flash.erase();
}

bool check = false;

void loop() {
    if (!check) {
        // 空いているところを探す
        uint32_t Addr = flash.setFlashAddress();
        // 書き込むデータを作る
        for (int i = 0; i < 256; i++) {
            tx[i] = i;
        }
        // 書き込み
        flash.write(Addr, tx);
        delay(1);
        // 読み込み
        flash.read(Addr, rx);
        // 読み込んだデータをシリアルで表示
        for (int i = 0; i < 256; i++) {
            Serial.print(rx[i]);
            Serial.print(" ");
        }
        check = true;
    }
}