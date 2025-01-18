# CREATE CAN Library

ESP32のtwaiライブラリを簡単につかえるようにしました。  

## 特徴
### arduino likeで簡単に使える
begin関数を実行したらあとは旧ライブラリとほとんど同じで簡単に使えます。
### 最大8文字まで同時に送信できる
sendLine関数では、charの配列を入力すればそのまま送信することができます。
### 通信相手とプロトコルを共通化できる
sendLine関数などを用いて複数文字を送信されると困る場合、相手と共通の設定を行えるためそもそも複数文字の送信ができないようにすることが可能です。
### エラーの返り値が詳細
成功時の返り値は0固定で、エラー時の返り値は1以上の値となり詳細なエラー内容を知ることが可能です。また、setupでSerial.beginが実行されているときエラー内容をシリアルに送信します。  
(旧ライブラリ互換動作時は旧ライブラリの返り値となります)
### 旧ライブラリのプログラムをそのまま使うこともできる
旧ライブラリの帰り値と同じにするモードを搭載しているため、旧ライブラリのプログラムをそのまま使うこともできます。(setPin、 begin、 available、 read、 sendPacket関数のみ対応)
### ESP32シリーズに対応
ESP32無印のみならず、ESP32 S3にも対応しています。また、他のESP32シリーズでも動作させられるはずです。
### Arduino ESP_IDF 両対応
ArduinoはもちろんESP_IDFでの動作が可能となっています。(動作確認はまだ済んでません)

## example
### 1文字しか送受信しない例
```cpp
#include <CAN.h>
#include <Arduino.h>

#define CAN_RX 17 // CAN ICのTXに接続しているピン
#define CAN_TX 18 // CAN ICのRXに接続しているピン

CAN_CREATE CAN(true); // 旧ライブラリ互換かどうか決める trueで新ライブラリ用になる

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);

  Serial.println("CAN Sender");
  // 100 kbpsでCANを動作させる
  if (CAN.begin(100E3, CAN_RX, CAN_TX, 10))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

void loop()
{
  if (CAN.available()) // CAN受信用
  {
    // CANを受信していたら実行される
    char Data;
    if (CAN.read(&Data))
    { // エラーの場合の処理
      Serial.println("failed to get CAN data");
    }
    else
      Serial.printf("Can received!!!: %c\n", Data);
  }
  if (Serial.available())
  {
    char cmd = Serial.read();
    Serial.println(cmd);
    if (CAN.sendChar(cmd))
    {
      Serial.println("failed to send CAN data");
    }
  }
}

```

### 2文字以上送信する例(uint8_t 配列を利用する場合)
```cpp
#include <CAN.h>
#include <Arduino.h>

#define CAN_RX 17 // CAN ICのTXに接続しているピン
#define CAN_TX 18 // CAN ICのRXに接続しているピン

CAN_CREATE CAN(true); // 旧ライブラリ互換かどうか決める trueで新ライブラリ用になる

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);

  Serial.println("CAN Sender");
  // start the CAN bus at 100 kbps
  if (CAN.begin(100E3, CAN_RX, CAN_TX, 10))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

void loop()
{
  if (CAN.available())
  {
    uint8_t Data[9]; // 最大8文字+改行文字が送信される
    if (CAN.readLine((char *)Data))
    {
      Serial.println("failed to get CAN data");
    }
    else
      Serial.printf("Can received!!!: %s\r\n", Data);
  }
  uint8_t Data[6] = {'c', 'r', 'e', 'a', 't', 'e'};
  CAN.sendData(Data, 6);

  delay(10);
}
```

### 2文字以上送信する例(char 文字列を利用する場合)
char文字列は、最後にnull文字(int型で0)が来ている必要があります。

```cpp
#include <CAN.h>
#include <Arduino.h>

#define CAN_RX 17 // CAN ICのTXに接続しているピン
#define CAN_TX 18 // CAN ICのRXに接続しているピン

C#include <CAN.h>
#include <Arduino.h>

#define CAN_RX 17 // CAN ICのTXに接続しているピン
#define CAN_TX 18 // CAN ICのRXに接続しているピン

CAN_CREATE CAN(true); // 旧ライブラリ互換かどうか決める trueで新ライブラリ用になる

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);

  Serial.println("CAN Sender");
  // start the CAN bus at 100 kbps
  if (CAN.begin(100E3, CAN_RX, CAN_TX, 10))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

void loop()
{
  if (CAN.available())
  {
    char Data[9]; // 最大8文字+改行文字が送信される
    if (CAN.readLine(Data))
    {
      Serial.println("failed to get CAN data");
    }
    else
      Serial.printf("Can received!!!: %s\r\n", Data);
  }
  if (Serial.available())
  {
    char data = Serial.peek(); // どの文字が入力されたかを確認(取り出さない)
    if (data == '\n' || data == '\r')
    {
      Serial.read(); // 改行文字だったら取り出して捨てる
      return;
    }
    char cmd[9] = {};
    for (int i = 0; i < 8; i++)
    {
      while (!Serial.available())
        ;
      char read = Serial.read();
      if (read == '\n' || read == '\r')
        break; // 改行文字だったら終了
      cmd[i] = read;
      Serial.print(read);
    }
    Serial.println();
    if (CAN.sendLine(cmd))
    {
      Serial.println("failed to send CAN data");
    }
  }
}
```

## TODO リスト
- common settingを作れるようにする
- idの0と1を予約済みにして、それぞれ自分のデータの送信やcommon settingの受け渡しに使えるようにしたい
- alertを使ってないから使う(エラー検出) status関数とかで実装する
- test関数の実装