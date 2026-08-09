#pragma once

#include <array>
#include <type_traits>

#include <AS5047D.h>
#include <CANCREATE.h>
#include <H3LIS331.h>
#include <I2CCREATE.h>
#include <ICM20602.h>
#include <ICM20948.h>
#include <ICM42688.h>
#include <LPS25HB.h>
#include <NEC920.h>
#include <S25FL127S.h>
#include <S25FL512S.h>
#include <SPICREATE.h>
#include <SSCDRRN005PD2A5.h>
#include <STS3215.h>
#include <STSCREATE.h>

inline void aviApiSmoke() {
  static_assert(!std::is_copy_constructible_v<SPICREATE>);
  static_assert(!std::is_copy_constructible_v<I2CCREATE>);
  static_assert(!std::is_copy_constructible_v<STSCREATE>);
  static_assert(!std::is_copy_constructible_v<STS3215>);
  static_assert(!std::is_copy_constructible_v<AS5047D>);
  static_assert(!std::is_copy_constructible_v<CANCREATE>);
  static_assert(!std::is_copy_constructible_v<ICM42688>);
  static_assert(!std::is_copy_constructible_v<ICM20602>);
  static_assert(!std::is_copy_constructible_v<ICM20948>);
  static_assert(!std::is_copy_constructible_v<LPS25HB>);
  static_assert(!std::is_copy_constructible_v<S25FL127S>);
  static_assert(!std::is_convertible_v<int, avi::Timeout>);
  static_assert(!std::is_convertible_v<uint32_t, avi::Timeout>);
  constexpr auto no_wait = avi::Timeout::noWait();
  constexpr auto one_ms = avi::Timeout::milliseconds(1);
  constexpr auto zero_ms = avi::Timeout::milliseconds(0);
  constexpr auto one_second = avi::Timeout::seconds(1);
  constexpr auto forever = avi::Timeout::forever();
  static_assert(no_wait.isNoWait() && zero_ms.isNoWait());
  static_assert(one_ms.isFinite() && one_second.isFinite());
  static_assert(forever.isForever());
  static_assert([] {
    uint64_t value{};
    return !avi::Timeout::seconds(UINT64_MAX).millisecondsValue(value);
  }());
  constexpr ICM42688::GyroRange gyro_ranges[]{
      ICM42688::GyroRange::dps15_625, ICM42688::GyroRange::dps31_25,
      ICM42688::GyroRange::dps62_5,   ICM42688::GyroRange::dps125,
      ICM42688::GyroRange::dps250,    ICM42688::GyroRange::dps500,
      ICM42688::GyroRange::dps1000,   ICM42688::GyroRange::dps2000};
  constexpr ICM42688::AccelOdr accel_odrs[]{
      ICM42688::AccelOdr::hz12_5,  ICM42688::AccelOdr::hz25,
      ICM42688::AccelOdr::hz50,    ICM42688::AccelOdr::hz100,
      ICM42688::AccelOdr::hz200,   ICM42688::AccelOdr::hz500,
      ICM42688::AccelOdr::hz1000,  ICM42688::AccelOdr::hz2000,
      ICM42688::AccelOdr::hz4000,  ICM42688::AccelOdr::hz8000,
      ICM42688::AccelOdr::hz16000, ICM42688::AccelOdr::hz32000};
  constexpr ICM42688::GyroOdr gyro_odrs[]{
      ICM42688::GyroOdr::hz12_5,  ICM42688::GyroOdr::hz25,
      ICM42688::GyroOdr::hz50,    ICM42688::GyroOdr::hz100,
      ICM42688::GyroOdr::hz200,   ICM42688::GyroOdr::hz500,
      ICM42688::GyroOdr::hz1000,  ICM42688::GyroOdr::hz2000,
      ICM42688::GyroOdr::hz4000,  ICM42688::GyroOdr::hz8000,
      ICM42688::GyroOdr::hz16000, ICM42688::GyroOdr::hz32000};
  (void)gyro_ranges;
  (void)accel_odrs;
  (void)gyro_odrs;

  // 不正GPIOと未初期化状態だけを使い、公開APIを実機I/Oなしでリンクする。
  SPICREATE spi;
  SPICREATE::Config spi_config{};
  spi_config.sck = -1;
  spi_config.transaction_timeout = one_ms;
  (void)spi.begin(spi_config);
  (void)spi.begin(SPI2_HOST, -1, 13, 11);
  (void)spi.deviceCount();
  (void)spi.end();

  I2CCREATE i2c;
  I2CCREATE::Config i2c_config{};
  i2c_config.sda = -1;
  (void)i2c.begin(i2c_config);
  (void)i2c.begin(I2C_NUM_0, -1, -1);
  (void)i2c.probe(0x28);
  (void)i2c.frequencyHz();
  (void)i2c.deviceCount();
  (void)i2c.end();

  AS5047D encoder;
  AS5047D::Config encoder_config{};
  AS5047D::RawData encoder_raw{};
  AS5047D::Data encoder_data{};
  AS5047D::Status encoder_status{};
  AS5047D::ErrorFlags encoder_errors{};
  (void)encoder.begin(spi, -1, encoder_config);
  (void)encoder.begin(spi, -1);
  (void)encoder.readRaw(encoder_raw);
  (void)encoder.read(encoder_data);
  (void)encoder.getStatus(encoder_status);
  (void)encoder.readAndClearErrorFlags(encoder_errors);
  (void)encoder.lastErrorFlags();
  (void)encoder.initialized();
  (void)encoder.end();

  CANCREATE can;
  CANCREATE::Config can_config{};
  CANCREATE::Frame can_frame{};
  CANCREATE::Status can_status{};
  CANCREATE::TestResult can_test{};
  uint8_t can_data[8]{};
  std::array<uint8_t, 8> can_array{};
  std::size_t available_count{};
  can_config.bitrate = CANCREATE::Bitrate::kbps500;
  (void)can.begin(can_config);
  (void)can.begin(GPIO_NUM_NC, GPIO_NUM_NC, CANCREATE::Bitrate::kbps500);
  (void)can.write(0x100, uint8_t{'s'});
  (void)can.write(0x100, can_data);
  (void)can.write(0x100, can_data, 4);
  (void)can.write(0x100, can_data, 4, avi::Timeout::milliseconds(10));
  (void)can.write(0x100, can_array);
  (void)can.writeText(0x100, "start");
  (void)can.write(can_frame);
  can_frame.identifier = CANCREATE::kApplicationIdMax;
  (void)can.write(can_frame);
  can_frame.identifier = CANCREATE::kDiagnosticIdMask;
  (void)can.write(can_frame);
  can_frame.extended = true;
  (void)can.write(can_frame);
  (void)can.read(can_frame);
  (void)can.available();
  (void)can.available(available_count);
  (void)can.getStatus(can_status);
  (void)can.recover();
  (void)can.test(can_test);
  (void)can.initialized();
  (void)can.end();

  ICM42688 icm42688;
  ICM42688::Config low_odr_config{};
  ICM42688::Config high_odr_config{};
  ICM42688::Data icm42688_data{};
  ICM42688::RawData icm42688_raw{};
  ICM42688::Status icm42688_status{};
  ICM42688::SelfTestResult icm42688_self_test{};
  bool ready{};
  low_odr_config.accel_odr = ICM42688::AccelOdr::hz1000;
  low_odr_config.gyro_odr = ICM42688::GyroOdr::hz1000;
  high_odr_config.accel_odr = ICM42688::AccelOdr::hz4000;
  high_odr_config.gyro_odr = ICM42688::GyroOdr::hz32000;
  high_odr_config.gyro_range = ICM42688::GyroRange::dps15_625;
  high_odr_config.int_gpio = GPIO_NUM_4;
  uint8_t identity{};
  (void)icm42688.begin(spi, -1, low_odr_config);
  (void)icm42688.begin(spi, -1, high_odr_config);
  (void)icm42688.begin(spi, -1);
  (void)icm42688.whoAmI(identity);
  (void)icm42688.getStatus(icm42688_status);
  (void)icm42688.available();
  (void)icm42688.available(ready);
  (void)icm42688.waitDataReady(no_wait);
  (void)icm42688.waitDataReady(one_ms);
  (void)icm42688.waitDataReady(one_second);
  (void)icm42688.waitDataReady(forever);
  (void)icm42688.readRaw(icm42688_raw);
  (void)icm42688.read(icm42688_data);
  (void)icm42688.selfTest(icm42688_self_test);
  (void)icm42688.initialized();
  (void)icm42688.end();

  ICM20948 icm20948;
  ICM20948::Config icm20948_config{};
  ICM20948::Data icm20948_data{};
  ICM20948::RawData icm20948_raw{};
  ICM20948::Status icm20948_status{};
  ICM20948::SelfTestResult icm20948_self_test{};
  icm20948_config.operation_timeout = avi::Timeout::milliseconds(300);
  (void)icm20948.begin(spi, -1, icm20948_config);
  (void)icm20948.begin(spi, -1);
  (void)icm20948.whoAmI(identity);
  (void)icm20948.getStatus(icm20948_status);
  (void)icm20948.available();
  (void)icm20948.readRaw(icm20948_raw);
  (void)icm20948.read(icm20948_data);
  (void)icm20948.selfTest(icm20948_self_test);
  (void)icm20948.initialized();
  (void)icm20948.end();

  LPS25HB pressure;
  LPS25HB::Config pressure_config{};
  LPS25HB::Data pressure_data{};
  LPS25HB::RawData pressure_raw{};
  LPS25HB::Status pressure_status{};
  LPS25HB::SpiConfig pressure_spi_config{};
  pressure_config.one_shot_timeout = avi::Timeout::milliseconds(100);
  (void)pressure.begin(spi, -1, pressure_config);
  (void)pressure.begin(spi, -1, pressure_spi_config, pressure_config);
  (void)pressure.begin(spi, -1);
  (void)pressure.begin(i2c, LPS25HB::Address::high, pressure_config);
  (void)pressure.begin(i2c, LPS25HB::Address::low);
  (void)pressure.whoAmI(identity);
  (void)pressure.getStatus(pressure_status);
  (void)pressure.available();
  (void)pressure.readRaw(pressure_raw);
  (void)pressure.read(pressure_data);
  (void)pressure.initialized();
  (void)pressure.end();

  SSCDRRN005PD2A5 ssc;
  SSCDRRN005PD2A5::RawData ssc_raw{};
  SSCDRRN005PD2A5::Data ssc_data{};
  (void)ssc.begin(i2c);
  (void)ssc.readRaw(ssc_raw);
  (void)ssc.read(ssc_data);
  (void)ssc.end();

  STSCREATE sts;
  STSCREATE::Config sts_config{};
  sts_config.tx = GPIO_NUM_NC;
  constexpr STSCREATE::Baudrate sts_baudrates[]{
      STSCREATE::Baudrate::bps1000000, STSCREATE::Baudrate::bps500000,
      STSCREATE::Baudrate::bps250000,  STSCREATE::Baudrate::bps128000,
      STSCREATE::Baudrate::bps115200,  STSCREATE::Baudrate::bps76800,
      STSCREATE::Baudrate::bps57600,   STSCREATE::Baudrate::bps38400};
  (void)sts_baudrates;
  uint8_t sts_data[8]{};
  uint8_t sts_error{};
  uint8_t sts_ids[]{1, 2};
  (void)sts.begin(sts_config);
  (void)sts.ping(1);
  (void)sts.read(1, 0x38, sts_data, 2);
  (void)sts.write(1, 0x2A, sts_data, 2);
  (void)sts.regWrite(1, 0x2A, sts_data, 2);
  (void)sts.action();
  (void)sts.action(1, true, &sts_error);
  (void)sts.action(1, false);
  (void)sts.action(0xFE, false);
  (void)sts.action(0xFE, true);
  (void)sts.syncRead(0x38, 2, sts_ids, 2, sts_data, 4);
  (void)sts.syncWrite(0x2A, 2, sts_ids, 2, sts_data, 4);
  (void)sts.recovery(1);
  (void)sts.recovery(1, &sts_error);
  (void)sts.recovery(1, nullptr, false);
  (void)sts.resetState(1);
  (void)sts.resetState(1, &sts_error);
  (void)sts.resetState(1, nullptr, false);

  STS3215 servo;
  STS3215::RawData servo_raw{};
  STS3215::Data servo_data{};
  STS3215::Status servo_status{};
  STS3215::Motion motion{1.0F, 1.0F, STS3215::TorqueLimit::percent(50.0F)};
  STS3215::StallProtection stall{true, 50.0F, 100, 20.0F};
  STS3215::TorqueLimit torque = STS3215::TorqueLimit::raw(500);
  STS3215::TorqueLimit invalid_raw = STS3215::TorqueLimit::raw(1001);
  STS3215::TorqueLimit invalid_percent = STS3215::TorqueLimit::percent(-1.0F);
  (void)STS3215::Model::c001_1_345;
  (void)STS3215::Model::c044_1_191;
  (void)STS3215::Model::c046_1_147;
  (void)servo.begin(sts, 1, STS3215::Model::c001_1_345);
  (void)servo.lastDeviceError();
  (void)torque.valid();
  (void)invalid_raw.valid();
  (void)invalid_percent.valid();
  (void)servo.verifyOperatingMode(STS3215::OperatingMode::step);
  (void)servo.holdCurrentPosition({torque});
  (void)servo.moveAbsoluteDegrees(10.0F, motion);
  (void)servo.moveRelativeDegrees(-10.0F, motion);
  (void)servo.enableTorque();
  (void)servo.disableTorque();
  (void)servo.setTorqueLimit(torque);
  (void)servo.readTorqueLimit(torque);
  (void)servo.configureStallProtection(stall,
                                       STS3215::Persistence::volatile_only);
  (void)servo.readRaw(servo_raw);
  (void)servo.read(servo_data);
  (void)servo.getStatus(servo_status);
  (void)servo.readRegister(STS3215::Register::id, sts_data, 1);
  (void)servo.writeRegister(STS3215::Register::id, sts_data, 1,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::baud_rate, sts_data, 1,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::response_status_level, sts_data,
                            1, STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::min_position_limit, sts_data, 2,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::max_position_limit, sts_data, 2,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::phase, sts_data, 1,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::angular_resolution, sts_data, 1,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::operating_mode, sts_data, 1,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::current_position, sts_data, 2,
                            STS3215::Persistence::volatile_only);
  (void)servo.writeRegister(STS3215::Register::servo_status, sts_data, 1,
                            STS3215::Persistence::volatile_only);
  (void)servo.end();
  (void)sts.end();

  S25FL127S flash;
  S25FL127S::Config flash_config{};
  S25FL127S::JedecId jedec_id{};
  S25FL127S::Status typed_flash_status{};
  uint8_t flash_status{};
  uint8_t buffer[S25FL127S::kPageSize]{};
  std::array<uint8_t, S25FL127S::kPageSize> flash_array{};
  (void)flash.begin(spi, -1, flash_config);
  (void)flash.begin(spi, -1);
  (void)flash.readJedecId(jedec_id);
  (void)flash.readStatus(flash_status);
  (void)flash.getStatus(typed_flash_status);
  (void)flash.blockSize();
  (void)flash.eraseBlock(0);
  (void)flash.eraseChip();
  (void)flash.write(0, buffer);
  (void)flash.write(0, buffer, 128);
  (void)flash.write(0, buffer, 128, avi::Timeout::milliseconds(1000));
  (void)flash.write(0, flash_array);
  (void)flash.read(0, buffer, sizeof(buffer));
  (void)flash.writeByte(0, 0);
  (void)flash.readByte(0, flash_status);
  (void)flash.initialized();
  (void)flash.end();

  ICM20602 icm20602;
  ICM20602::Config icm20602_config{};
  ICM20602::RawData icm20602_raw{};
  ICM20602::SelfTestResult icm20602_self_test{};
  ICM20602::Data icm20602_data{};
  ICM20602::Status icm20602_status{};
  icm20602_config.accel_dlpf = ICM20602::AccelDlpf::hz44_8;
  icm20602_config.gyro_dlpf = ICM20602::GyroDlpf::hz41;
  (void)icm20602.begin(spi, -1, icm20602_config);
  (void)icm20602.begin(spi, -1);
  (void)icm20602.whoAmI(identity);
  (void)icm20602.available();
  (void)icm20602.getStatus(icm20602_status);
  (void)icm20602.readRaw(icm20602_raw);
  (void)icm20602.read(icm20602_data);
  (void)icm20602.selfTest(icm20602_self_test);
  (void)icm20602.initialized();
  (void)icm20602.end();
  H3LIS331 h3lis331;
  S25FL512S flash512;
  NEC920 radio;
  NEC920::Packet radio_packet{};
  std::array<uint8_t, 4> radio_destination{};
  bool radio_accepted{};
  uint8_t radio_data[]{0x73};
  (void)radio.send(NEC920::MessageId::no_resend, 1, radio_destination,
                   radio_data, 0);
  (void)radio.setRfConfig(1, 1, 1, 1, 1, 0);
  (void)radio.receive(radio_packet, 0);
  (void)radio.checkCommandResult(radio_packet, 1, radio_accepted);
  (void)radio.available();
  (void)h3lis331;
  (void)flash512;
}
