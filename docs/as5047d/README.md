# AS5047D

AS5047Dは1-frameのcommand-response pipelineを持ち、frame NのMISOは同じframeのMOSI commandではなく、1つ前のcommandへのresponseです。通常の`read()`/`readRaw()`は、最初のframeでregister readを送り、そのstale responseはparityだけ検査します。次のframeでNOP read（`0x0000`）を送り、ここでrequested registerのdata/EFを評価するため、1回の角度取得に2つの16-bit SPI frameを使います。

高速連続取得では、最初に`startPipelinedRead()`でANGLE read commandをprimeします。このprime frameのresponseも以前のcommandに対応するためEFをANGLE commandの結果として扱いません。その後は`readPipelinedRaw()`または`readPipelined()`を呼ぶたびにANGLE commandを送りつつ、前回commandのANGLE responseを1 frame/sampleで取得します。

```cpp
AS5047D::Config config{};
config.frequency_hz = 10000000;
ESP_ERROR_CHECK(encoder.begin(spi, 10, config));
ESP_ERROR_CHECK(encoder.startPipelinedRead());

AS5047D::RawData data{};
ESP_ERROR_CHECK(encoder.readPipelinedRaw(data));
```

pipeline中は`read()`、`readRaw()`、`getStatus()`、`readAndClearErrorFlags()`を使用できず、`ESP_ERR_INVALID_STATE`を返します。通常register accessへ戻す場合は`stopPipelinedRead()`を呼びます。`end()`はpipeline中でも実行でき、pipeline stateを破棄します。

SPI transport error、response parity error、requested commandに対応するEF検出のいずれかが起きた場合、commandとresponseの対応を安全に保証できないためpipelineは自動的に無効になります。エラー原因を処理した後、必要なら`startPipelinedRead()`で再primeしてください。EF時は`ERRFL`を2 frameで取得してread-to-clearし、PARERR、INVCOMM、FRERRを`lastErrorFlags()`へ保存します。ERRFLは取得時にhardware上でclearされるため、あとから`readAndClearErrorFlags()`を呼ぶと全flagが0の場合があります。直前のEF原因には`lastErrorFlags()`を使用してください。

このAPIはsensor FIFOではなく、過去sampleを蓄積しません。10 kHzで利用する場合、100 us周期そのものはapplication側で生成してください。timer ISRから直接SPIを呼ばず、高優先度taskを起床してtask contextから`readPipelinedRaw()`を呼ぶ構成を推奨します。同一AS5047D instanceを複数taskから同時操作する場合のserializeも呼出し側の責務です。

`pipelined_example.cpp`に10 MHz SPIでの最小構成を示します。
