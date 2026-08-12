# ICM42688

`ICM42688` は通常register snapshot readとFIFO readを別APIとして扱います。実装基準はTDK InvenSense `DS-000347 ICM-42688-P v1.6`です。

## Data Ready

INT GPIOを指定してFIFOを無効にした場合、ISRはsample sequenceをincrementしてstatic semaphoreを通知します。semaphoreはtaskを起こすためのhintであり、freshnessの判定は`produced != consumed`のsequence差で行います。

- `available()`は未消費sampleの有無を確認し、sampleを消費しません。
- `waitDataReady()`は未消費sampleが存在するまで待ちますが、sampleを消費しません。
- `read()` / `readRaw()`が読出し開始前までのsequenceを消費します。
- SPI読出し中に次のIRQが発生した場合、そのsequenceは消費せずfreshのまま残します。

INT GPIOを使わない場合は`INT_STATUS.DATA_RDY_INT`をpollingします。通常registerは各sensorの最新snapshotなので、Accel/Gyroへ異なるODRを設定した場合、遅い側の値が複数回同じになることがあります。各sampleの生成時刻と連続性が必要な用途ではFIFOを使用してください。

## FIFO formatとwatermark

現在は16-byte Packet 3のみを扱います。

- Accel 16-bit x3
- Gyro 16-bit x3
- Temperature 8-bit
- ODR timestamp delta 16-bit
- internal clock、`TMST_RES=0`、`TMST_DELTA_EN=1`

ICM-42688-Pの物理FIFOは2048 bytesです。16-byte Packet 3では最大128 recordsなので、`watermark_records`は`1..128`だけを受け付けます。2080 bytesのdriver bufferはread cacheを含むhost側の確保量であり、sensor FIFO容量としては扱いません。

## Timestamp epoch

`begin()`ではAccel/GyroとFIFO設定をsensor OFF中に完了し、sensorをONにしてgyro起動時間45 msを待ちます。その後、起動待ち中に生成されたFIFO sampleだけを`FIFO_FLUSH`で一度破棄し、直後を`timestamp_us = 0`のepochとして開始します。同時に`FIFO_LOST_PKT_CNT`のbaselineを取得します。

このstartup flushは意図的なepoch開始操作です。runtimeでは自動flushしません。

## FIFO full時の扱い

`FIFO_FULL_INT`またはwatermark到達は「readを急ぐ状態」として扱います。`waitFifo()`はFIFO count、threshold、fullのいずれかでreadyになりますが、fullだけを理由に`FIFO_FLUSH`、timestamp state reset、lost-packet baseline resetは行いません。

理由は次のとおりです。

1. `FIFO_FULL_INT`と`FIFO_LOST_PKT_CNT`はdatasheet上で別の状態です。fullになっただけでは、softwareがまだ読めるrecordまで捨てる必要はありません。
2. `FIFO_FLUSH`はFIFO内容を明示的に破棄する操作です。runtimeで自動実行すると、library自身がtimestamp continuityを切断します。
3. 現在の`timestamp_us`はPacket 3のODR deltaを累積して作ります。失われたpacketのdeltaを取得できないため、packet loss後の連続時刻をlibraryが推測して埋めることはしません。

`readFifoRaw()` / `readFifo()`はburst readの前後で`FIFO_LOST_PKT_CNT`を確認します。epoch開始時のbaselineから値が変化した場合、またはread中に値が変化した場合はcontinuity lossとしてinstanceを`fifo_faulted_`状態にし、異常batchを返しません。

fault後は`getFifoStatus()`で状態を診断できます。timestampを暗黙に0へ戻して処理を継続することはせず、`end()`してから再度`begin()`して新しいepochを開始します。これにより、連続していないtimestampを連続値として扱うことを避けます。

## 同期要件

同一`ICM42688` instanceへの複数taskからの同時操作はサポートしません。`read()`、FIFO read、self-test、`end()`などは呼出し側でserializeしてください。異なるSPI device instance間のtransactionは`SPICREATE`がserializeします。
