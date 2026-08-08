# Avi_ESP_Libs

ESP32向けのSPI、TWAI/CAN、センサ、Flash、UARTドライバを、リポジトリ全体で1つのC++17ライブラリとして提供します。PlatformIOでは1つのlibrary、ESP-IDFでは1つのcomponentとして利用できます。CIのtargetはESP32-S3です。

## 対応環境と固定バージョン

| 環境 | 固定内容 | 内包framework |
| --- | --- | --- |
| PlatformIO Core | `6.1.19` | - |
| PlatformIO Arduino | `espressif32@7.0.1` | Arduino-ESP32 `2.0.17`（package `3.20017.241212+sha.dcc1105b`）/ ESP-IDF `4.4.7` |
| PlatformIO ESP-IDF | `espressif32@7.0.1` | ESP-IDF `6.0.1`（package `framework-espidf 4.60001.0`） |
| 純ESP-IDF stable | `espressif/idf:v6.0.2` | ESP-IDF `6.0.2` |
| 純ESP-IDF latest | `espressif/idf:latest` | ESP-IDF master追従 |

PlatformIOの2環境と純ESP-IDF stable/latestをsmoke buildします。ESP-IDF `latest`だけは将来の非互換を検出する目的で意図的に固定していません。

## 初心者向けAPI

同じSPI busへ複数deviceを登録しても、利用側がESP-IDF handleを管理する必要はありません。

```cpp
SPICREATE spi;
ICM42688 imu;
LPS25HB pressure;
S25FL127S flash;

spi.begin(SPI2_HOST, 12, 13, 11);
imu.begin(spi, 10);
pressure.begin(spi, 9);
flash.begin(spi, 8);

CANCREATE can;
can.begin(GPIO_NUM_18, GPIO_NUM_17, CANCREATE::Bitrate::kbps500);
can.write(0x100, uint8_t{'s'});

if (can.available()) {
    CANCREATE::Frame frame;
    can.read(frame);
}
```

Tier 1 sensorは `begin()`、`available()`、`read()`、`end()` が基本です。`read()`は単位付きの物理値、`readRaw()`はdevice registerの符号付き整数を返します。

## ドライバの区分

Tier 1は今回、設定、初期化確認、測定または通信、状態取得、有限timeout、終了処理を重点整備した対象です。

| Tier 1 | 主な機能 |
| --- | --- |
| `SPICREATE` | SPI busの所有、最大8 deviceの共有、有限timeout、初期化・終了状態の検査 |
| `CANCREATE` | Classic TWAI frame、標準/拡張ID filter、3 mode、状態取得、bus-off復旧 |
| `ICM42688` | 加速度/角速度range、ODR、filter、INT GPIO、Data Ready待機 |
| `ICM20602` | 加速度/角速度range、sample divider、DLPF、Data Ready状態 |
| `ICM20948` | 加速度/角速度range、sample divider、DLPF、AK09916 ODR、9軸測定 |
| `LPS25HB` | ODR、圧力/温度average、one-shot、ready/overrun状態、物理値変換 |
| `S25FL127S` | JEDEC/typed status、範囲検査、page分割write、read、block/chip erase |

Tier 2は`H3LIS331`、`S25FL512S`、`NEC920`です。ESP32-S3実機では未検証であり、各公開ヘッダは`#pragma message("TODO: ... Tier 2 ...")`を表示します。NEC920はraw UARTに加えて、固定長packet送受信、RF設定、command応答判定を保持します。

Tier 1もCIでは実機へ接続しないため、実デバイスでの電気的・機能的検証は別途必要です。

## ディレクトリ構成

```text
.
├── .github/workflows/build.yml  # 4環境のCI
├── include/                     # 公開ヘッダ
│   └── avi_esp_libs/            # 共通version/compatibility宣言
├── src/
│   ├── bus/
│   ├── compatibility/
│   ├── radio/
│   ├── sensor/
│   └── storage/
├── docs/                        # 現行APIの文書とexample
├── test_apps/                   # 3環境のsmoke project
├── CMakeLists.txt               # ESP-IDF component登録
├── idf_component.yml
├── library.json                 # PlatformIO package metadata
└── library.properties           # Arduino library metadata
```

`legacy/`へ残す現行候補はありません。MCP2562FD、LogBoard67、Log67Timer、Log67SerialはESP32-S3向けpackageから削除済みです。

## PlatformIOから使う

リポジトリを依存に指定します。再現可能なbuildではbranch名ではなくcommit SHAまたはrelease tagを固定してください。

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32@7.0.1
board = esp32-s3-devkitc-1
framework = arduino
lib_deps =
    https://github.com/CREATE-ROCKET/Avi_ESP_Libs.git#<commit-or-tag>
build_unflags = -std=gnu++11
build_flags = -std=gnu++17
```

ESP-IDF frameworkを使う場合は`framework = espidf`へ変更します。ローカル開発では`lib_deps = symlink://../..`または`file://`依存を利用できます。`lib_extra_dirs`は不要です。

## ESP-IDF componentとして使う

projectの`components/Avi_ESP_Libs`へこのリポジトリを配置します。利用側componentから依存を宣言します。

```cmake
idf_component_register(SRCS "main.cpp" REQUIRES Avi_ESP_Libs)
target_compile_options(${COMPONENT_LIB} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>")
```

別の配置場所を使う場合は、project側でその場所を`EXTRA_COMPONENT_DIRS`へ追加してください。利用側でESP-IDFのSPI device handleを管理する必要はありません。

## 基本API

```cpp
#include <ICM20948.h>
#include <SPICREATE.h>

SPICREATE spi;
ICM20948 imu;

esp_err_t initialize()
{
    const esp_err_t spi_result = spi.begin(SPI2_HOST, 12, 13, 11);
    if (spi_result != ESP_OK) {
        return spi_result;
    }

    ICM20948::Config config;
    config.accel_range = ICM20948::AccelRange::g8;
    config.gyro_range = ICM20948::GyroRange::dps1000;

    const esp_err_t imu_result = imu.begin(spi, 10, config);
    if (imu_result != ESP_OK) {
        (void)spi.end();
        return imu_result;
    }
    return ESP_OK;
}

esp_err_t measure(ICM20948::Data &data)
{
    return imu.read(data);
}
```

既定値でよければ`imu.begin(spi, 10)`だけで開始できます。詳細設定は各classの`Config`を変更して`begin(spi, cs, config)`へ渡します。deviceを先に`end()`し、その後に共有する`SPICREATE`を`end()`してください。

sensorの物理値fieldは次の単位です。

- `acceleration_g`: g
- `angular_velocity_dps`: degree/second
- `temperature_celsius`: degree Celsius
- `magnetic_ut`: microtesla
- `pressure_pa`: pascal

ICM42688のData Ready割込みは次のように利用します。

```cpp
ICM42688 imu42688;

esp_err_t initializeDataReady()
{
    ICM42688::Config config;
    config.int_gpio = GPIO_NUM_4;
    return imu42688.begin(spi, 10, config);
}

esp_err_t measureWhenReady(ICM42688::Data &data)
{
    const esp_err_t result = imu42688.waitDataReady(100);
    if (result != ESP_OK) {
        return result;
    }
    return imu42688.read(data);
}
```

ISRはsemaphore通知だけを行います。SPI通信、動的確保、ログ、blocking処理はISR内で行わず、暗黙のbackground taskも生成しません。

CANの対応bitrateは`kbps25`、`kbps50`、`kbps100`、`kbps125`、`kbps250`、`kbps500`、`kbps800`、`mbps1`です。11bit standard frameのbyte/buffer送信には`write(identifier, ...)`、extended/RTRには`Frame` APIを使います。Classic CANのため8 byte超過は拒否します。`read(Frame&)`の既定timeoutは0 msです。

ICM20948はbank切替を伴うため、同一instanceを複数taskから同時に呼び出さないでください。他のdeviceも、同一instanceの`read()`/`write()`と`end()`/再設定は呼出し側でserializeしてください。異なるdevice instance間のSPI transactionは`SPICREATE`がserializeします。

S25FL127Sの`write()`はpage境界を内部処理しますが、自動eraseはしません。NOR Flashで0から1へ戻す領域は、先にaligned 64 KiB `eraseBlock()`または`eraseChip()`で消去してください。FL-Sの4 KiB parameter sectorは配置が構成依存のため、誤消去を避けて汎用`eraseSector()` APIを設けていません。

## エラーと所有権

- 回復可能なエラーは`esp_err_t`で呼出し元へ返します。
- ライブラリ内部の通常エラー処理に`ESP_ERROR_CHECK`、`abort()`、restart、永久待機を使いません。
- 所有権を持つclassはcopy/moveを禁止しています。
- 二重初期化、未初期化利用、無効GPIO、`nullptr`、範囲外、timeout、部分初期化失敗、二重解放を検査します。
- driverが作成したESP-IDF handleだけを解放します。
- `SPICREATE`を利用するdeviceより先に破棄しないでください。

## CI

`.github/workflows/build.yml`は次のtriggerで動作します。

- 全branchへの`push`
- `workflow_dispatch`
- 毎月1日00:00 UTCのschedule

jobはPlatformIO Arduino、PlatformIO ESP-IDF、ESP-IDF stable `v6.0.2`、ESP-IDF `latest`の4つです。stableは再現性のため完全なDocker tagを固定し、Espressifの公式stable更新を確認して手動更新します。`latest`はmaster追従で、失敗を許容しません。uploadは行いません。

smoke appは全公開ヘッダを同じtranslation unitで読み込み、Tier 1の公開APIを実機I/Oなしでコンパイル・リンクします。

## 破壊的変更

この版は旧`main`および旧バージョン番号付きdirectoryとの後方互換性がありません。

- `CAN_CREATE`、`CANCREATE_lib.h`、旧互換mode、`setPin()`、`sendChar()`、`sendData()`、`sendLine()`、`readLine()`、`sendPacket()`を削除しました。
- `CANCREATE::Config::bitrate`と簡易`begin()`は任意整数から`Bitrate` enumへ変更し、`read(Frame&)`の既定timeoutを0 msへ変更しました。
- `SPICREATE::SPICreate`と、利用側が触れていた`addDevice()`等の内部APIを削除しました。
- 曖昧な公開型`ICM`、`Flash`、`LPS`を廃止し、device名と同じclass名へ統一しました。互換aliasはありません。
- Tier 1の戻り値、`Config`、`Data`、`Status`を`esp_err_t`中心のAPIへ変更しました。
- Tier 1 sensorの`get()`を廃止し、raw整数の`readRaw()`と物理値の`read()`へ分離しました。
- S25FL127Sの曖昧な`erase()`を`eraseChip()`へ変更しました。
- MCP2562FDおよび67系専用libraryを削除しました。

移行時は各公開ヘッダの宣言を基準に呼出し側を更新してください。
