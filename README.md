# Avi_ESP_Libs

ESP32向けのSPI、I2C、STS servo、TWAI/CAN、センサ、Flash、UARTドライバを、リポジトリ全体で1つのC++17ライブラリとして提供します。PlatformIOでは1つのlibrary、ESP-IDFでは1つのcomponentとして利用できます。CIのtargetはESP32-S3です。

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
AS5047D encoder;
ICM42688 imu;
LPS25HB pressure;
S25FL127S flash;

spi.begin(SPI2_HOST, 12, 13, 11);
encoder.begin(spi, 7);
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

I2CCREATE i2c;
SSCDRRN005PD2A5 differential_pressure;
i2c.begin(I2C_NUM_0, 8, 9, 400000);
differential_pressure.begin(i2c);

STSCREATE sts_bus;
STS3215 servo;
STSCREATE::Config sts_config;
sts_config.tx = GPIO_NUM_17;
sts_config.rx = GPIO_NUM_18;
sts_bus.begin(sts_config);
servo.begin(sts_bus, 1, STS3215::Model::c001_1_345); // 動作は開始しない
```

Tier 1 sensorは `begin()`、`available()`、`read()`、`end()` が基本です。`read()`は単位付きの物理値、`readRaw()`はdevice registerの符号付き整数を返します。

待機時間は整数ではなく`avi::Timeout`で明示します。

```cpp
avi::Timeout::noWait();          // 0 ms
avi::Timeout::milliseconds(20); // 有限待機
avi::Timeout::seconds(1);       // 有限待機
avi::Timeout::forever();        // 明示した場合だけ無限待機
```

CAN queue、Data Ready、SPI bus lockなど、利用者が任意に待つAPIの既定値は`noWait()`です。準備前なら`ESP_ERR_NOT_FINISHED`、有限待機の期限切れなら`ESP_ERR_TIMEOUT`を返します。一方、sensor reset、one-shot、Flash program/eraseなどを成立させる内部期限は故障検出に必要なので、Configに有限の既定値を持ちます。これらのoperation timeoutは`noWait()`と`forever()`を拒否します。

## ドライバの区分

Tier 1は今回、設定、初期化確認、測定または通信、状態取得、有限timeout、終了処理を重点整備した対象です。

| Tier 1 | 主な機能 |
| --- | --- |
| `SPICREATE` | SPI busの所有、最大8 deviceの共有、有限timeout、初期化・終了状態の検査 |
| `I2CCREATE` | I2C bus共有、Repeated START、IDF 4.4 legacy/IDF 6 new driver、lock/operation timeout |
| `STSCREATE` | Feetech STS packet、half-duplex方向制御、checksum、同期read/write |
| `CANCREATE` | Classic TWAI frame、標準/拡張ID filter、起動時diagnostic、bus-off復旧 |
| `AS5047D` | 14-bit角度、補償/未補償値、parity/ERRFL、磁界diagnostic |
| `ICM42688` | 全加速度/角速度range、accel/gyro別ODR、filter、INT GPIO、Data Ready待機 |
| `ICM20602` | 加速度/角速度range、sample divider、accel/gyro別DLPF、Data Ready状態 |
| `ICM20948` | 加速度/角速度range、sample divider、DLPF、AK09916 ODR、9軸測定 |
| `LPS25HB` | SPI/I2C transport、ODR、圧力/温度average、one-shot、ready/overrun状態、物理値変換 |
| `SSCDRRN005PD2A5` | ±5 psi differential pressure、14-bit圧力、11-bit温度、SSC status |
| `STS3215` | position/step movement、現在位置保持、runtime torque、stall protection、telemetry |
| `S25FL127S` | JEDEC/typed status、範囲検査、page分割write、read、block/chip erase |

Tier 2は`H3LIS331`、`S25FL512S`、`NEC920`です。ESP32-S3実機では未検証であり、各公開ヘッダは`#pragma message("TODO: ... Tier 2 ...")`を表示します。NEC920はraw UARTに加えて、固定長packet送受信、RF設定、command応答判定を保持します。

Tier 1もCIでは実機へ接続しないため、実デバイスでの電気的・機能的検証は別途必要です。

今回追加したI2CCREATE、LPS25HB I2C、SSCDRRN005PD2A5、STSCREATE、STS3215はTier 1 APIとして設計・build検証済みですが、ESP32-S3実機では未検証です。

### I2C transport

`I2CCREATE`はbus clockを共有し、device driverからのtransaction全体をmutexでserializeします。`lock_timeout`は`noWait()`、有限時間、`forever()`に対応し、`operation_timeout`はbus故障を検出する有限deadlineです。register readはIDF 4.4の`i2c_master_write_read_device()`またはIDF 6の`i2c_master_transmit_receive()`を使い、writeとreadの間へSTOPを入れません。

LPS25HBは同じclassをSPIまたはI2C address `0x5C`/`0x5D`で開始できます。I2C transportではreset、one-shotを含めて`CTRL_REG2.I2C_DISABLE`を設定しません。SSCDRRN005PD2A5は4-byte packetから14-bit圧力と11-bit温度を取得し、normal以外のstatusを`RawData`へ残します。物理値readではstaleを`ESP_ERR_NOT_FINISHED`、command modeを`ESP_ERR_INVALID_STATE`、diagnostic faultを`ESP_ERR_INVALID_RESPONSE`として返します。

### STS3215

`STSCREATE`は磁気エンコーダ版STS protocolのlittle-endian packet層です。PING、READ、WRITE、REG WRITE、ACTION、SYNC READ/WRITE、RECOVERY、状態resetを提供します。broadcast PINGは衝突を避けるため拒否します。direction pin指定時はUART shift registerの送信完了後、delayを挟まずRXへ切り替えます。`lock_timeout`はbus mutex取得、`tx_timeout`はUART shift registerの送信完了、`response_timeout`は1 response packet全体の受信deadlineです。tx/responseのdefaultは各100 msです。

公開する最低baudrate 38.4 kbpsでは、8N1の259-byte packetの理論wire timeが約67.5 ms（切上げ68 ms）になるため、従来の共通10 ms defaultでは最大packetを送信できません。1 Mbpsで短いpacketだけを使う場合は`tx_timeout`と`response_timeout`を明示的に10 ms等へ短縮できます。SYNC READの1台あたりのpayloadは1～253 byteです。SYNC READ/WRITEはpacket size、data size、ID重複を送信前に検査します。`syncRead()`のoptional `device_errors`配列には各IDのresponse ERROR byteが入ります。途中で失敗した場合、それ以前に正常受信したdata/error entryは更新済みで、失敗したID以降は未変更です。

`esp_err_t`はUART transportとpacket framingの成否を表し、responseの`ERROR` byteはservoが報告する別の状態です。したがって`ESP_OK`かつ`device_error != 0`は正常なprotocol transactionです。`STS3215`はresponseを受信したtransactionの値を`lastDeviceError()`へ保存し、fault中でも`getStatus()`やtelemetryを取得できます。responseを待たないWRITEでは保存値を変更しません。

`STS3215::begin()`はPINGと設定snapshotだけを読み、torque、target、operating mode、EPROMを変更しないためservoは動きません。cache対象はresponse level、angular resolution、operating mode、phase、position limitsです。全項目の読取とvalidationが成功した場合だけ一括反映され、`configurationValid()`がtrueになります。`refreshConfiguration()`で実機から再取得でき、途中失敗または不正値ではcacheをinvalidにします。typed configuration setterはwriteの成否にかかわらずread-backし、実機値とcacheを再同期します。

cache invalid時はmode確認、typed configuration、movement、`holdCurrentPosition()`、物理値`read(Data&)`を`ESP_ERR_INVALID_STATE`で拒否します。`degreesPerStep()`はquiet NaNを返します。一方、安全停止と診断のため`disableTorque()`、`enableTorque()`、runtime torque、`readRaw()`、`getStatus()`、generic read、`lastDeviceError()`は利用できます。`holdCurrentPosition()`はposition modeでは現在位置をtargetへ設定し、step modeではrelative target 0を設定してからruntime torque limit、torque ONの順に適用します。既にtorqueが有効でも旧targetへ力を掛けないよう、targetを先に合わせます。Torque Switchのcalibration値128は使用しません。`disableTorque()`後は機械的条件が許せば手で回せ、再度`holdCurrentPosition()`するとその現在位置を保持します。

`moveRelativeDegrees()`はcurrent positionのread+加算ではなく、step mode nativeのBIT15方向 + 15-bit magnitudeを使います。speedはPhase BIT2に応じて1または50 steps/s単位、accelerationは100 steps/s²単位へ変換します。movementごとの`Motion::torque_limit`、SRAM runtime torque、EPROM stall protectionは別の設定です。

telemetryのposition、speed、currentはBIT15、loadはBIT10を符号とするsign-magnitudeです。`Data`ではposition/speed/currentを符号付き物理値へ変換し、loadは符号付きraw値を返します。`TorqueLimit::raw()`/`percent()`は範囲外、NaN、無限値に対して`valid()==false`となり、受付APIは`ESP_ERR_INVALID_ARG`を返します。stall protectionの`trigger_time_ms`は0～2540 msをnearest 10 msへ量子化するservo設定値で、待機を表す`avi::Timeout`ではありません。

EPROM setterは`Persistence`を要求し、lock flagを一時変更して必ずbest-effortで元へ戻します。ID、baudrate、response level、position limit、phase、angular resolution、operating modeはinstanceの通信条件またはcacheへ影響するため、generic `writeRegister()`では`ESP_ERR_NOT_SUPPORTED`として意図的に禁止します。cacheを変更する設定はhardwareとcacheを同期するtyped setterだけを使用します。current positionとservo statusもread-onlyとしてgeneric writeを拒否します。typed `Register` accessはregister固有widthだけを許可するため、隣接するcache-sensitive/read-only registerへの越境writeはできません。任意spanのraw accessは提供せず、高レベルmovement内部の連続WRITEだけを個別に管理します。

提供された磁気エンコーダ版memory tableに従い、Angular Resolutionは`1..3`のみ有効で、`degrees_per_step = 360 / 4096 * resolution`です。同資料でServo Status 0x41のBIT4は未定義のためtyped boolを設けず、値は`Status::raw`へ保持します。broadcast ACTIONはresponseを返さないため、broadcast IDと`wait_response=true`の組合せは`ESP_ERR_INVALID_ARG`です。複数servo instanceは同じSTSCREATEを共有できますが、同一STS3215 instanceの設定、movement、telemetry、`lastDeviceError()`の利用は呼出し側でserializeしてください。

### AS5047D

AS5047DはSPI mode 1、最大10 MHzで動作し、各16-bit frame間に2 usのCSn HIGH時間を確保します。14-bit角度をdegree/radianへ変換し、`AngleSource`で動的角度誤差補償済み`ANGLECOM`と未補償`ANGLEUNC`を選択できます。全responseのeven parityとEFを検査し、EF時はread-to-clearの`ERRFL`からPARERR、INVCOMM、FRERRを`lastErrorFlags()`へ保存します。

`getStatus()`はDIAAGC/MAGからMAGL、MAGH、COF、offset compensation完了、AGC、magnitudeを返します。磁界警告は通信errorへ変換しません。永久変更を伴うOTP programmingには対応していません。

### ICM self-test

`ICM42688`、`ICM20602`、`ICM20948`の`selfTest()`はMEMSのself-test stimulusを有効化し、各deviceのvendor手順に従ってbaselineとstimulated sampleをfactory trimと比較します。

- ICM42688はvendor実装どおりresponseの絶対値を使います。factory code有効時はaccel/gyroとも50%超・150%未満、gyro baselineは20 dps以下です。factory code欠損時はaccel 50～1200 mg、gyro 60 dps以上を使用します。
- ICM20602はfactory code有効時に符号付きresponseを使い、accelは50%超・150%未満、gyroは50%超、gyro baselineは20 dps以下です。factory code欠損時だけresponseの絶対値を使い、accel 225～675 mg、gyro 60 dps以上で判定します。
- ICM20948は符号付きresponseを使うため、逆方向responseを絶対値化してPASSさせません。factory code欠損はFAIL、accelは50～150%、gyroは50%以上です。AK09916はX/Yを-200～+200、Zを-1000～-200で判定します。

`ESP_OK`は手順が正常に完了した意味であり、個体の合否は`SelfTestResult::passed`で確認します。self-test中は通常測定を行わず、同一instanceの操作は呼出し側でserializeしてください。終了時は元のConfigを復元し、`restored`で結果を示します。

self-test中は通常測定できません。同一instanceの`read()`、`waitDataReady()`、`end()`や別taskからの操作は、呼出し側で停止・serializeしてください。

### CAN diagnostic

standard ID `0x000`～`0x3FF`はapplication用、`0x400`～`0x7FF`はCANCREATE diagnostic予約領域、`0x7FF`は`test()`用です。extended IDは予約規則の対象外です。

`test()`は現在のbitrate/GPIOでnormal single-shot送信のACKを確認し、失敗時だけno-ack/self-receptionを試します。診断timeoutは内部で1秒に固定しています。`success`、`no_peer_response`、`controller_failure`を`TestResult`で返し、元Configへ復元します。起動時専用で、test中の通常trafficは保持されない可能性があります。複数nodeから同時に実行しないでください。

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
    const esp_err_t result =
        imu42688.waitDataReady(avi::Timeout::milliseconds(100));
    if (result != ESP_OK) {
        return result;
    }
    return imu42688.read(data);
}
```

ISRはstatic semaphore通知だけを行います。INT GPIO設定時の`available()`と`waitDataReady()`はSPI pollingを行わず、`available()`は通知を消費せず、`waitDataReady()`だけが消費します。`read()`と`readRaw()`は通知状態を変更せず、現在のregisterをsnapshotとして読みます。SPI通信、動的確保、ログ、blocking処理はISR内で行わず、暗黙のbackground taskも生成しません。

同期して読む場合は、次のどちらかを一貫して使用してください。

```cpp
if (imu42688.available() &&
    imu42688.waitDataReady(avi::Timeout::noWait()) == ESP_OK) {
    imu42688.read(data);
}

imu42688.waitDataReady(avi::Timeout::forever());
imu42688.read(data);
```

`read()`または`readRaw()`を直接呼ぶ使い方は非同期snapshot readです。直接readと`waitDataReady()`を混在させた場合、通知とsampleの対応は保証しません。

### ICM42688 FIFO

通常の`read()`/`readRaw()`は従来どおり最新のsensor registerを読み、FIFOを消費しません。FIFOはdefault disabledの別APIで、DS-000347 v1.6の16-byte Packet 3（Accel + Gyro + 8-bit Temperature + ODR Timestamp）のみを扱います。

```cpp
ICM42688::Config config{};
config.accel_odr = ICM42688::AccelOdr::hz1000;
config.gyro_odr = ICM42688::GyroOdr::hz1000;
config.int_gpio = GPIO_NUM_4;
config.fifo.enabled = true;
config.fifo.watermark_records = 4;

esp_err_t result = imu42688.begin(spi, 10, config);
std::array<ICM42688::FifoData, 16> samples{};
std::size_t count{};
if (result == ESP_OK &&
    imu42688.waitFifo(avi::Timeout::milliseconds(10)) == ESP_OK) {
    result = imu42688.readFifo(samples.data(), samples.size(), count);
}
```

`timestamp_ticks`はpacket内のraw ODR delta、`timestamp_us`はFIFO開始epochからのsensor-relative時刻です。internal clock、`TMST_RES=0`のためv1.6 section 12.7どおり32/30を整数remainder付きで累積し、ESP32のtimer epochやSPI read時刻とは一致しません。最初のpacketにもsensorのdeltaを反映します。`getFifoStatus()`でwatermark、FIFO full、lost packet数を確認できます。full/lossは通信errorへ変換せずstatusで通知します。

FIFO有効時はAccel/Gyro ODRを同一かつ12.5 Hz～2 kHzにしてください。4 kHz以上は45 msのgyro startup中に2048-byte FIFO容量を超えるため`begin()`で拒否します。FIFOの複合readと同じinstanceの他操作は呼出し側でserializeしてください。INT GPIO使用時はDATA_RDYをrouteせず、FIFO threshold/fullだけをstatic semaphoreへ通知します。`waitFifo()`はFIFO countを先に確認し、semaphoreはwake-up hintとしてのみ使用します。ISRはSPI、heap、logging、blockingを行いません。Packet 4 high-resolution、FSYNC、external RTC、Accel-only、Gyro-only、異なるAccel/Gyro ODRには対応しません。Accel/Gyroの`-32768`はv1.6のinvalid markerとしてvalidity flagをfalseにし、通信成功とは区別します。FIFO温度についてv1.6は独立したinvalid markerを規定していないため、対応構成では`temperature_valid=true`です。

加速度またはジャイロのODRが4 kHz以上でINT GPIOを使う場合、driverはdatasheetの要件に従い`INT_CONFIG1.INT_TPULSE_DURATION`と`INT_TDEASSERT_DISABLE`を自動設定します。4 kHz未満ではこれらを解除します。`INT_ASYNC_RESET`は全ODRで解除します。

ICM42688は加速度・ジャイロごとに12.5 Hzから32 kHzまでのLow Noise ODRを指定でき、ジャイロrangeは±2000、1000、500、250、125、62.5、31.25、15.625 dpsを選べます。ICM20602は`AccelDlpf`と`GyroDlpf`を別々に設定し、それぞれ`ACCEL_CONFIG2`と`CONFIG`へ反映します。

CANの対応bitrateは`kbps25`、`kbps50`、`kbps100`、`kbps125`、`kbps250`、`kbps500`、`kbps800`、`mbps1`です。11bit standard frameのbyte/buffer送信には`write(identifier, ...)`、extended/RTRには`Frame` APIを使います。Classic CANのため8 byte超過は拒否します。`read(Frame&)`の既定timeoutは0 msです。

ICM20948はbank切替を伴うため、同一instanceを複数taskから同時に呼び出さないでください。他のdeviceも、同一instanceの`read()`/`write()`と`end()`/再設定は呼出し側でserializeしてください。異なるdevice instance間のSPI transactionは`SPICREATE`がserializeします。

S25FL127Sの`write()`はpage境界を内部処理しますが、自動eraseはしません。NOR Flashで0から1へ戻す領域は、先に`eraseBlock()`または`eraseChip()`で消去してください。`begin()`はSR2の`D8h_O`を読み、D8h block erase単位を64 KiBまたは256 KiBとして自動検出します。現在値は`blockSize()`で取得できます。FL-Sの4 KiB parameter sectorは配置が構成依存のため、誤消去を避けて汎用`eraseSector()` APIを設けていません。

## エラーと所有権

- 回復可能なエラーは`esp_err_t`で呼出し元へ返します。
- ライブラリ内部の通常エラー処理に`ESP_ERROR_CHECK`、`abort()`、restart、永久待機を使いません。
- 所有権を持つclassはcopy/moveを禁止しています。
- 二重初期化、未初期化利用、無効GPIO、`nullptr`、範囲外、timeout、部分初期化失敗、二重解放を検査します。
- driverが作成したESP-IDF handleだけを解放します。
- `SPICREATE`を利用するdeviceより先に破棄しないでください。
- `I2CCREATE`/`STSCREATE`も接続deviceより先に破棄しないでください。

## CI

`.github/workflows/build.yml`は次のtriggerで動作します。

- 全branchへの`push`
- `workflow_dispatch`
- 毎月1日00:00 UTCのschedule

jobはPlatformIO Arduino、PlatformIO ESP-IDF、ESP-IDF stable `v6.0.2`、ESP-IDF `latest`の4つです。stableは再現性のため完全なDocker tagを固定し、Espressifの公式stable更新を確認して手動更新します。`latest`はmaster追従で、失敗を許容しません。uploadは行いません。

smoke appは全公開ヘッダを同じtranslation unitで読み込み、Tier 1の公開APIを実機I/Oなしでコンパイル・リンクします。

## 破壊的変更

この版は旧`main`および旧バージョン番号付きdirectoryとの後方互換性がありません。

- `STS3215::Data::load_raw`を`uint16_t`からsign-magnitude復号済みの`int16_t`へ変更しました。
- `STS3215::StallProtection::trigger_time`を`avi::Timeout`から`uint16_t trigger_time_ms`へ変更しました。
- cache-sensitive registerとread-only telemetryのgeneric `writeRegister()`は`ESP_ERR_NOT_SUPPORTED`を返します。
- `STSCREATE::Config`へ`tx_timeout`を追加し、`response_timeout`のdefaultを10 msから100 msへ変更しました。
- `STSCREATE::syncRead()`へsource-compatibleなoptional `device_errors`末尾引数を追加しました。
- `STS3215`へ`configurationValid()`と`refreshConfiguration()`を追加し、cache invalid時の高レベル操作を拒否します。`degreesPerStep()`はcache invalid時にNaNを返します。
- `STS3215::readRegister()`/`writeRegister()`は各`Register`固有width以外を拒否します。SYNC READ/WRITEも表現不能なsizeと重複IDを送信前に拒否します。

- `CAN_CREATE`、`CANCREATE_lib.h`、旧互換mode、`setPin()`、`sendChar()`、`sendData()`、`sendLine()`、`readLine()`、`sendPacket()`を削除しました。
- `CANCREATE::Config::bitrate`と簡易`begin()`は任意整数から`Bitrate` enumへ変更し、`read(Frame&)`の既定timeoutを0 msへ変更しました。
- standard CAN ID `0x400`～`0x7FF`をdiagnostic予約領域とし、applicationの`read()`/`write()`から除外しました。
- 公開timeout引数を`uint32_t`から暗黙整数変換できない`avi::Timeout`へ変更し、利用者待機の既定値を`noWait()`へ統一しました。
- no-waitで未完了の場合は`ESP_ERR_TIMEOUT`ではなく`ESP_ERR_NOT_FINISHED`を返します。
- ICM20602の`Dlpf`を`AccelDlpf`と`GyroDlpf`へ分離しました。
- ICM42688の共通`Odr`を`AccelOdr`と`GyroOdr`へ分離し、range/ODRを拡張しました。
- S25FL127Sの固定`kBlockSize`を削除し、実機設定を返す`blockSize()`へ変更しました。
- `SPICREATE::SPICreate`と、利用側が触れていた`addDevice()`等の内部APIを削除しました。
- 曖昧な公開型`ICM`、`Flash`、`LPS`を廃止し、device名と同じclass名へ統一しました。互換aliasはありません。
- Tier 1の戻り値、`Config`、`Data`、`Status`を`esp_err_t`中心のAPIへ変更しました。
- Tier 1 sensorの`get()`を廃止し、raw整数の`readRaw()`と物理値の`read()`へ分離しました。
- S25FL127Sの曖昧な`erase()`を`eraseChip()`へ変更しました。
- LPS25HBのmeasurement `Config`からSPI `frequency_hz`を削除し、`SpiConfig`へ分離しました。I2C begin overloadを追加しています。
- MCP2562FDおよび67系専用libraryを削除しました。

移行時は各公開ヘッダの宣言を基準に呼出し側を更新してください。
