#include <ICM42688.h>
#include <SPICREATE.h>

#include <array>
#include <cstdio>

namespace {
SPICREATE spi;
ICM42688 imu;
uint32_t batches{};
} // namespace

esp_err_t beginFifo(spi_host_device_t host, int sck, int miso, int mosi,
                    int chip_select, gpio_num_t interrupt_gpio) {
  esp_err_t result = spi.begin(host, sck, miso, mosi, 2081);
  if (result != ESP_OK)
    return result;
  ICM42688::Config config{};
  config.accel_odr = ICM42688::AccelOdr::hz1000;
  config.gyro_odr = ICM42688::GyroOdr::hz1000;
  config.int_gpio = interrupt_gpio;
  config.fifo.enabled = true;
  config.fifo.watermark_records = 4;
  result = imu.begin(spi, chip_select, config);
  if (result != ESP_OK)
    (void)spi.end();
  return result;
}

esp_err_t readFifoOnce() {
  esp_err_t result = imu.waitFifo(avi::Timeout::milliseconds(10));
  if (result != ESP_OK)
    return result;

  ICM42688::FifoStatus status{};
  result = imu.getFifoStatus(status);
  if (result != ESP_OK)
    return result;

  std::array<ICM42688::FifoData, 16> samples{};
  std::size_t count{};
  result = imu.readFifo(samples.data(), samples.size(), count);
  if (result != ESP_OK) {
    std::printf("FIFO parse/read error: %s\n", esp_err_to_name(result));
    return result;
  }

  // UART負荷を抑えるため、100 batchごとに先頭sampleだけ表示する。
  if (count != 0 && (++batches % 100) == 0) {
    const auto &sample = samples[0];
    std::printf("fifo=%u read=%u full=%u lost=%u ticks=%u timestamp_us=%llu "
                "accel=%.3f,%.3f,%.3f gyro=%.3f,%.3f,%.3f temp=%.2f\n",
                static_cast<unsigned>(status.records_available),
                static_cast<unsigned>(count), status.full,
                static_cast<unsigned>(status.lost_packets),
                static_cast<unsigned>(sample.timestamp_ticks),
                static_cast<unsigned long long>(sample.timestamp_us),
                sample.acceleration_g[0], sample.acceleration_g[1],
                sample.acceleration_g[2], sample.angular_velocity_dps[0],
                sample.angular_velocity_dps[1], sample.angular_velocity_dps[2],
                sample.temperature_celsius);
  }
  return ESP_OK;
}
