# AS5047D

AS5047Dは1-frameのcommand-response pipelineを持ち、frame NのMISOは同じframeのMOSI commandではなく、1つ前のcommandへのresponseです。通常の`read()`/`readRaw()`は、最初のframeでregister readを送り、そのstale responseはparityだけ検査します。次のframeでNOP register read（address `0x0000`、wire command `0xC000`）を送り、ここでrequested registerのdata/EFを評価するため、1回の角度取得に2つの16-bit SPI frameを使います。

transportはSPI mode 1、最大10 MHz、16-bit MSB first、ESP-IDF hardware CSです。CSn fallingからfirst clockまで350 ns以上となるsetup cycle数を周波数から切り上げ計算し、8 MHzでは3 cycle（375 ns）、10 MHzでは4 cycle（400 ns）を設定します。last clockからCSn risingまで1 cycle、frame間CSn HIGHは2 usです。`SPICREATE`のbus mutexは各frameのCS assertからdeassertまで別deviceのtransactionが割り込まないことを保証します。

`begin()`はdatasheetのpower-on時間10 ms後にERRFLを1回read-to-clearし、その後のANGLE responseで初期化成功を確認します。startup errorを隠すretryは行いません。途中で失敗した場合はdeviceをremoveし、cleanupの失敗もcallerへ返します。removeに失敗したhandleは破棄せず、次回`begin()`でcleanupを再試行します。

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

SPI transport error、response parity error、requested commandに対応するEF検出のいずれかが起きた場合、commandとresponseの対応を安全に保証できないためpipelineは自動的に無効になります。エラー原因を処理した後、必要なら`startPipelinedRead()`で再primeしてください。EF時は`ERRFL`を2 frameで取得してread-to-clearし、PARERR、INVCOMM、FRERRを`lastErrorFlags()`へ保存します。response parityまたはERRFL PARERRは`ESP_ERR_INVALID_CRC`、INVCOMM/FRERRは`ESP_ERR_INVALID_RESPONSE`です。

EFは常にSPI framing errorを意味しません。ERRFLが0ならDIAAGCとZPOSLを追加取得し、ZPOSL設定でEFへ寄与するMAGH/MAGL、COF、未完了のoffset compensationをsensor diagnostic fault (`ESP_ERR_INVALID_STATE`) として分類します。それらでも説明できないEFは`ESP_ERR_INVALID_RESPONSE`です。ERRFLは取得時にhardware上でclearされるため、あとから`readAndClearErrorFlags()`を呼ぶと全flagが0の場合があります。直前のEF原因には`lastErrorFlags()`を使用してください。

このAPIはsensor FIFOではなく、過去sampleを蓄積しません。10 kHzで利用する場合、100 us周期そのものはapplication側で生成してください。timer ISRから直接SPIを呼ばず、高優先度taskを起床してtask contextから`readPipelinedRaw()`を呼ぶ構成を推奨します。同一AS5047D instanceを複数taskから同時操作する場合のserializeも呼出し側の責務です。

`pipelined_example.cpp`に10 MHz SPIでの最小構成を示します。
