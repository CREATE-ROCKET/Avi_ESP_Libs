# Avi_ESP_Libs

ESP32向けのバス、センサ、Flash、無線ドライバを、リポジトリ全体で1つのライブラリとして提供します。CI対象はESP32-S3です。

## 対応環境

- PlatformIO + Arduino
- PlatformIO + ESP-IDF
- 純粋なESP-IDF component

`include/`が公開ヘッダ、`src/`が実装、`test_apps/`が3環境のsmoke project、`docs/`が旧文書です。現在、現行ESP32-S3向けに残す必要があるコードはすべて移植済みで、`legacy/`対象はありません。削除済みの67系およびMCP2562FD旧実装は現行packageに含まれません。

## 導入

PlatformIOではGit URLまたはローカルの`symlink://`/`file://` URLを`lib_deps`へ指定します。platformは再現性のため`espressif32@7.0.1`を推奨します。

ESP-IDFではこのリポジトリをprojectの`components/Avi_ESP_Libs`へ配置するか、同名symlinkを作成します。ルート`CMakeLists.txt`がcomponentを登録します。

## 基本API

```cpp
#include <ICM20948.h>
#include <SPICREATE.h>

SPICREATE spi;
ICM20948 imu;

void initialize()
{
    const esp_err_t spi_result = spi.begin(SPI2_HOST, 12, 13, 11);
    if (spi_result != ESP_OK) {
        return;
    }

    const esp_err_t imu_result = imu.begin(spi, 10);
    if (imu_result != ESP_OK) {
        return;
    }
}
```

各driverはオブジェクトを生成し、`begin()`後に`get()`または`read()`/`write()`で利用します。エラーは`esp_err_t`で呼出し元へ返します。ライブラリ内部は通常エラー処理に`ESP_ERROR_CHECK`、`abort()`、永久待機を使いません。

## 公開モジュール

`SPICREATE`、`CANCREATE`、`ICM42688`、`ICM20948`、`ICM20602`、`H3LIS331`、`LPS25HB`、`S25FL127S`、`S25FL512S`、`NEC920`を提供します。

## CI

push、手動実行、毎月1日00:00 UTCにPlatformIO Arduino、PlatformIO ESP-IDF、ESP-IDF stable `v6.0.2`、ESP-IDF `latest`をbuildします。stable tagは公式stable更新時に手動更新します。`latest`はmaster追従で、将来の非互換もfailureとして検出します。

## 互換性

この再編は破壊的変更です。旧バージョン番号付きディレクトリ、曖昧な`ICM`/`Flash`/`LPS`型、旧APIとの後方互換性はありません。
