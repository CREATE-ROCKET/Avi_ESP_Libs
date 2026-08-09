# AS5047D

通常の`read()`/`readRaw()`は1回の角度取得につき2つの16-bit SPI frameを使います。AS5047Dのcommand-response pipelineを利用する高速連続取得では、最初に`startPipelinedRead()`でANGLE read commandをprimeし、その後は`readPipelinedRaw()`または`readPipelined()`を呼ぶたびに1つの16-bit frameで前回commandのANGLE responseを取得します。

```cpp
AS5047D::Config config{};
config.frequency_hz = 10000000;
ESP_ERROR_CHECK(encoder.begin(spi, 10, config));
ESP_ERROR_CHECK(encoder.startPipelinedRead());

AS5047D::RawData data{};
ESP_ERROR_CHECK(encoder.readPipelinedRaw(data));
```

pipeline中は`read()`、`readRaw()`、`getStatus()`、`readAndClearErrorFlags()`を使用できず、`ESP_ERR_INVALID_STATE`を返します。通常register accessへ戻す場合は`stopPipelinedRead()`を呼びます。`end()`はpipeline中でも実行でき、pipeline stateを破棄します。

SPI transport error、response parity error、EF検出のいずれかが起きた場合、commandとresponseの対応を安全に保証できないためpipelineは自動的に無効になります。エラー原因を処理した後、必要なら`startPipelinedRead()`で再primeしてください。EF時は既存どおり`ERRFL`をread-to-clearし、`lastErrorFlags()`へPARERR、INVCOMM、FRERRを保存します。

このAPIはsensor FIFOではなく、過去sampleを蓄積しません。10 kHzで利用する場合、100 us周期そのものはapplication側で生成してください。timer ISRから直接SPIを呼ばず、高優先度taskを起床してtask contextから`readPipelinedRaw()`を呼ぶ構成を推奨します。同一AS5047D instanceを複数taskから同時操作する場合のserializeも呼出し側の責務です。

`pipelined_example.cpp`に10 MHz SPIでの最小構成を示します。
