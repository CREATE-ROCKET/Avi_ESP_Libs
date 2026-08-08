# CANCREATE

ESP32のClassic TWAI/CAN controllerをArduinoとESP-IDFから同じAPIで使うTier 1 driverです。CAN FDには対応しません。

```cpp
CANCREATE can;

esp_err_t initializeCan()
{
    return can.begin(GPIO_NUM_18, GPIO_NUM_17,
                     CANCREATE::Bitrate::kbps500);
}

esp_err_t sendCan()
{
    const uint8_t command[]{0x73, 0x01};
    return can.write(0x100, command);
}

esp_err_t pollCan(CANCREATE::Frame &frame)
{
    return can.available() ? can.read(frame) : ESP_ERR_NOT_FINISHED;
}
```

`write(identifier, ...)`は11bit standard frame専用です。8 byte超過や0x7FFを超えるIDは拒否します。extended frame、RTR、DLCを指定する場合は`Frame`を使います。

```cpp
CANCREATE::Frame frame;
frame.identifier = 0x18FF50E5;
frame.extended = true;
frame.data_length = 2;
frame.data[0] = 0xCA;
frame.data[1] = 0xFE;
can.write(frame, 100);
```

詳細設定では`Config`へ`Bitrate`、normal/no-ack/listen-only mode、標準/拡張ID filter、RX queue depthを指定できます。bitrateは`kbps25`、`kbps50`、`kbps100`、`kbps125`、`kbps250`、`kbps500`、`kbps800`、`mbps1`です。

`available(std::size_t&)`、`getStatus()`、`recover()`はエラーを区別する詳細APIです。引数なし`available()`はエラー時に0を返す便利版です。`read(Frame&)`の既定timeoutは非blockingの0 msです。
