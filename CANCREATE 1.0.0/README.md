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
ArduinoはもちろんESP_IDFでの動作が可能となっています。

## 利用方法
以下のexampleをコピーして利用することや、vscode等利用時は関数にカーソルを合わせることででるホバーにその関数の使い方を載せてあるため、それを使うこともできます。

## 注意点
### シリアルにデバッグログを流したくないとき
デフォルトの動作ではSerial0が利用可能時自動的にログを出力しますが、ログが多すぎるなど利用したくない場合はplatformio.iniに以下の内容を追記する必要があります
```ini
build_flags = -DDEBUG_CAN=0
```
### 旧ライブラリ互換で利用したいとき
旧ライブラリ互換で動作する場合、setPin、 begin、 available、 read、 sendPacket関数の一部のみ対応となっていることに注意してください。(新規利用は非推奨)  
また、その場合コンストラクタに以下のようにfalseを渡す必要があります。
```cpp
CAN_CREATE CAN(false);
```

### IDの制限

CANは11bitのIDが利用できますが、最上位bitが1のときにはブロックされる設定となっています。

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
  if (CAN.sendData(Data, 6)){
    Serial.println("failed to send CAN data");
  }

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

### 送信ステータスを得られるようにした例

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
    Serial.println("receiving data...");
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

    // CREATE_CANライブラリが裏で処理を終わらせるための待ち時間 statusを見るために必要
    delay(100);
    switch (CAN.getStatus())
    {
    case CAN_SUCCESS:
      Serial.println("Success to Send!!!");
      break;
    case CAN_NO_ALERTS:
      Serial.println("CREATE_CAN driver isn't done yet");
      break;
    case CAN_BUS_ERROR:
      Serial.println("Got a bus error on the CAN such as ACK Error");
      break;
    case CAN_TX_FAILED:
      Serial.println("Can't send data, something other than a bus error might wrong");
      break;
    default:
      Serial.println("Unknown error occurred!!!");
      break;
    }
  }
}
```

## test関数を利用する例
test関数を利用することで通信ができない場合に自分のコントローラーが動いているかを確かめることができ、原因の絞り込みが可能となります。  
ただし、この関数で得られた結果は必ずしも正しいとは限らないことに注意してください。
```cpp
#include <CAN.h>
#include <CAN_lib.h>

#include <Arduino.h>

#define CAN_RX 17 // CAN ICのTXに接続しているピン
#define CAN_TX 18 // CAN ICのRXに接続しているピン

CAN_CREATE CAN(true); // 旧ライブラリ互換かどうか決める trueで新ライブラリ用になる

can_setting_t can_setting = {
    ((long)100E3),
    false,
    CAN_FILTER_DEFAULT,
};

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("CAN Sender");
  // 100 kbpsでCANを動作させる
  if (CAN.begin(can_setting, CAN_RX, CAN_TX, 10))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  delay(100);
  switch (CAN.test())
  {
  case CAN_SUCCESS:
    Serial.println("Success!!!"); // 通信成功
    break;
  case CAN_UNKNOWN_ERROR:
    Serial.println("Unknown error occurred"); // 不可逆なエラーが発生した可能性がある 再起動推奨
    break;
  case CAN_NO_RESPONSE_ERROR:
    Serial.println("No response error"); // 自分のコントローラーは動作している
    break;
  case CAN_CONTROLLER_ERROR:
    Serial.println("CAN CONTROLLER ERROR"); // 自分のコントローラーが動作していない
    break;
  default:
    break;
  }
}

void loop()
{

  if (CAN.available())
  {
    Serial.println("receiving data...");
    can_return_t Data;
    if (CAN.readWithDetail(&Data))
    {
      Serial.println("failed to get CAN data");
    }
    else
    {
      Serial.printf("CAN received!!!\n id:\t %u \n size: \t %d \n data: \t %.*s",
                    Data.id,
                    Data.size,
                    Data.size,
                    Data.data
                    );
    }
  }

  if (Serial.available())
  {
    char cmd = Serial.read();
    Serial.println(cmd);
    if (CAN.sendChar(10, cmd))
    {
      Serial.println("failed to send CAN data");
    }
  }
}
```

## can_setting_tを利用し、id等の正確なデータを得る例

```cpp
#include <CANCREATE.h>
#include <Arduino.h>

#define CAN_RX 17 // CAN ICのTXに接続しているピン
#define CAN_TX 18 // CAN ICのRXに接続しているピン

CAN_CREATE CAN(true); // 旧ライブラリ互換かどうか決める trueで新ライブラリ用になる

can_setting_t Settings = {
    .baudRate = (long)100E3,
    .multiData_send = true,
    .filter_config = CAN_FILTER_DEFAULT,
};

int i;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);

  Serial.println("CAN Sender");
  // start the CAN bus at 100 kbps
  if (CAN.begin(Settings, CAN_RX, CAN_TX, 10))
  {
    Serial.println("Starting CAN failed!");
    while (true)
      ;
  }
  i = 0;
}

void loop()
{
  if (CAN.available())
  {
    can_return_t Data;
    if (CAN.readWithDetail(&Data))
    {
      Serial.println("failed to get CAN data");
    }
    else
    {
      // 得られたデータを出力する %.*s フォーマット指定子は直前の引数を参照してその文字数だけ出力する
      Serial.printf("CAN received!!!\r\n id:\t %u \r\n size: \t %d \r\n data: \t %.*s\r\n",
                    Data.id,
                    Data.size,
                    Data.size,
                    Data.data);
    }
    if (i != Data.id)
    {
      Serial.printf("failed at %d, found %d", i, Data.id);
    }
    Serial.println(i);
    i++;
  }
  if (Serial.available())
  {
    for (int i = 0; i < (1 << 29); i++)
    {
      CAN.sendChar(i, 'a');
      delay(100);
    }
  }
}
```

---
## 昔のモードを利用する例
> [!WARNING]
> 旧ライブラリ用として書かれたプログラムをそのまま利用できるようにしたものです。 新規に作る場合に採用は非推奨です。

注意点
- CANのコンストラクタでfalse値を渡す必要がある
- Arduino.hはライブラリ内でincludeしないため、インクルードが必要
```cpp
// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>
#include <Arduino.h>

CAN_CREATE CAN(false);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("CAN Receiver");
  CAN.setPins(17, 18);

  // start the CAN bus at 100 kbps
  if (!CAN.begin(100E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

void loop()
{
  // try to parse packet

  if (CAN.available())
  {
    char cmd = (char)CAN.read();
    Serial.print(cmd);
  }
  if (Serial.available())
  {
    char data = Serial.read();
    Serial.println(data);
    CAN.sendPacket(0x13, data);
    int result = CAN.sendChar(data); // no supported
    if (result)
    {
      Serial.print(result);
    }
  }
}
```