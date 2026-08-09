#include <STS3215.h>
#include <STSCREATE.h>

STSCREATE bus;
STS3215 servo;

esp_err_t initializeServo() {
  STSCREATE::Config config{};
  config.tx = GPIO_NUM_17;
  config.rx = GPIO_NUM_18;
  esp_err_t result = bus.begin(config);
  if (result == ESP_OK)
    result = servo.begin(bus, 1, STS3215::Model::c001_1_345);
  // begin()は設定snapshotを読むだけでservoを動かさない。
  return result;
}

esp_err_t moveAndRelease() {
  esp_err_t result = servo.verifyOperatingMode(STS3215::OperatingMode::step);
  if (result == ESP_OK) {
    // relative target 0、runtime torque limit、torque
    // ONの順で現在位置を保持する。
    result = servo.holdCurrentPosition({STS3215::TorqueLimit::percent(30.0F)});
  }
  STS3215::Motion motion{};
  motion.speed_deg_s = 90.0F;
  motion.acceleration_deg_s2 = 360.0F;
  motion.torque_limit = STS3215::TorqueLimit::percent(50.0F);
  if (result == ESP_OK)
    result = servo.moveRelativeDegrees(130.0F, motion);
  STS3215::Status status{};
  if (result == ESP_OK)
    result = servo.getStatus(status); // fault中でも状態を取得できる。
  if (result == ESP_OK) {
    const uint8_t response_error = servo.lastDeviceError();
    (void)response_error; // transport成功とservoのERROR byteは別に扱う。
  }
  if (result == ESP_OK)
    result = servo.disableTorque(); // 手で回せる状態にする。
  if (result == ESP_OK) {
    result = servo.holdCurrentPosition(
        {STS3215::TorqueLimit::percent(30.0F)}); // 現在位置を保持する。
  }
  return result;
}
