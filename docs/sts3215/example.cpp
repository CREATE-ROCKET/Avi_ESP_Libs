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
  // begin()は設定を読むだけでservoを動かさない。
  return result;
}

esp_err_t moveAndRelease() {
  esp_err_t result = servo.verifyOperatingMode(STS3215::OperatingMode::step);
  if (result == ESP_OK) {
    result = servo.holdCurrentPosition({STS3215::TorqueLimit::percent(30.0F)});
  }
  STS3215::Motion motion{};
  motion.speed_deg_s = 90.0F;
  motion.acceleration_deg_s2 = 360.0F;
  motion.torque_limit = STS3215::TorqueLimit::percent(50.0F);
  if (result == ESP_OK)
    result = servo.moveRelativeDegrees(130.0F, motion);
  if (result == ESP_OK)
    result = servo.disableTorque(); // 手で回せる状態にする。
  if (result == ESP_OK) {
    result = servo.holdCurrentPosition(
        {STS3215::TorqueLimit::percent(30.0F)}); // 現在位置を保持する。
  }
  return result;
}
