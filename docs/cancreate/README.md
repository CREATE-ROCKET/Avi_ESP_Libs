# CANCREATE

ESP32のClassic TWAI/CAN controllerを、ArduinoとESP-IDFから同じAPIで利用するTier 1 driverです。CAN FDには対応しません。

```cpp
#include <CANCREATE.h>

CANCREATE can;

esp_err_t initializeCan()
{
    CANCREATE::Config config;
    config.tx = GPIO_NUM_18;
    config.rx = GPIO_NUM_17;
    config.bitrate = 500000;

    config.filter.enabled = true;
    config.filter.identifier = 0x120;
    config.filter.mask = 0x7F0;

    return can.begin(config);
}

esp_err_t sendCan()
{
    CANCREATE::Frame frame;
    frame.identifier = 0x123;
    frame.data_length = 2;
    frame.data[0] = 0xCA;
    frame.data[1] = 0xFE;
    return can.write(frame, 100);
}

esp_err_t receiveCan(CANCREATE::Frame &frame)
{
    return can.read(frame, 100);
}
```

`Config::mode`でnormal、no-ack、listen-onlyを選択できます。`getStatus()`はqueue数、error counter、bus stateを返し、bus-off後は`recover()`で有限時間の復旧を要求できます。

`CAN_CREATE`、constructorによる旧互換mode、`setPin()`、文字列送受信helperは削除済みです。複数byteは`Frame::data`と`data_length`を直接指定してください。旧生成済みDoxygen HTMLは`legacy-html/`に保存していますが、現行APIの資料ではありません。
