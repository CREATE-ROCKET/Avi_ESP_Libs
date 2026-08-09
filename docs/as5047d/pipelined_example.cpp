#include <AS5047D.h>
#include <SPICREATE.h>

SPICREATE spi;
AS5047D encoder;

esp_err_t initializeEncoder() {
  const esp_err_t spi_result = spi.begin(SPI2_HOST, 12, 13, 11);
  if (spi_result != ESP_OK)
    return spi_result;

  AS5047D::Config config{};
  config.frequency_hz = 10000000;
  const esp_err_t encoder_result = encoder.begin(spi, 10, config);
  if (encoder_result != ESP_OK) {
    (void)spi.end();
    return encoder_result;
  }

  return encoder.startPipelinedRead();
}

esp_err_t sampleAngle(AS5047D::RawData &data) {
  // 呼出し側で100 us周期を生成する。ISRからSPI readは行わない。
  return encoder.readPipelinedRaw(data);
}

esp_err_t stopEncoder() {
  if (encoder.pipelinedReadActive()) {
    const esp_err_t result = encoder.stopPipelinedRead();
    if (result != ESP_OK)
      return result;
  }
  const esp_err_t encoder_result = encoder.end();
  if (encoder_result != ESP_OK)
    return encoder_result;
  return spi.end();
}
