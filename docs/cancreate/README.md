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

`write(identifier, ...)`は11bit standard frame専用です。application IDは`0x000`～`0x3FF`です。`0x400`～`0x7FF`はCANCREATE内部diagnostic用に予約し、通常送受信から除外します。extended frameは予約規則の対象外です。8 byte超過も拒否します。

```cpp
CANCREATE::Frame frame;
frame.identifier = 0x18FF50E5;
frame.extended = true;
frame.data_length = 2;
frame.data[0] = 0xCA;
frame.data[1] = 0xFE;
can.write(frame, avi::Timeout::milliseconds(100));
```

詳細設定では`Config`へ`Bitrate`、normal/no-ack/listen-only mode、標準/拡張ID filter、RX queue depthを指定できます。bitrateは`kbps25`、`kbps50`、`kbps100`、`kbps125`、`kbps250`、`kbps500`、`kbps800`、`mbps1`です。

`available(std::size_t&)`、`getStatus()`、`recover()`はエラーを区別する詳細APIです。引数なし`available()`はエラー時に0を返す便利版です。`read()`、`write()`、`recover()`の既定timeoutは`avi::Timeout::noWait()`です。no-waitでqueueが空または満杯なら`ESP_ERR_NOT_FINISHED`、有限待機が期限切れなら`ESP_ERR_TIMEOUT`を返します。`avi::Timeout::forever()`は明示した場合だけ永久待機します。

## 起動時診断

`test()`は予約ID `0x7FF`のsingle-shot frameをnormal modeで送信し、ACKが得られれば`success`を返します。ACKが得られなければno-ack/self-receptionを試し、成功時は`no_peer_response`、失敗時は`controller_failure`です。診断手順が完了した場合は判定FAILでも`ESP_OK`であり、driver操作や復元の失敗だけを`esp_err_t`で返します。

```cpp
CANCREATE::TestResult test_result{};
const esp_err_t error = can.test(test_result);
```

起動時、通常trafficを開始する前に使用してください。診断timeoutはライブラリ内部で1秒に固定され、呼出し側から変更できません。test中はqueueを含むbackendを一時再生成するため既存trafficが保持されない可能性があります。複数nodeから同時に実行すると判定が不安定になります。終了時は元の`Config`へ復元し、復元失敗時は`restored=false`としてCANを未初期化状態にします。
