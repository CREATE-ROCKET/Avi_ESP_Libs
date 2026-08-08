#include <ICM42688.h>
#include <SPICREATE.h>

SPICREATE spi;
ICM42688 imu;
bool ready = false;

void setup() {
  if (spi.begin(SPI2_HOST, 14, 12, 13) != ESP_OK) {
    return;
  }

  ICM42688::Config config;
  config.accel_range = ICM42688::AccelRange::g8;
  config.gyro_range = ICM42688::GyroRange::dps1000;
  config.accel_odr = ICM42688::AccelOdr::hz200;
  config.gyro_odr = ICM42688::GyroOdr::hz200;
  config.filter = ICM42688::Filter::odr_div4;
  config.int_gpio = GPIO_NUM_4;

  if (imu.begin(spi, 15, config) != ESP_OK) {
    (void)spi.end();
    return;
  }
  ready = true;
}

void loop() {
  if (!ready || imu.waitDataReady(avi::Timeout::milliseconds(100)) != ESP_OK) {
    return;
  }

  ICM42688::Data data;
  if (imu.read(data) != ESP_OK) {
    return;
  }

  // acceleration_g、angular_velocity_dps、temperature_celsiusを利用する。
}
