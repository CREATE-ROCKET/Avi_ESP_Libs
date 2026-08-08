#pragma once

#include <type_traits>

#include <CANCREATE.h>
#include <H3LIS331.h>
#include <ICM20602.h>
#include <ICM20948.h>
#include <ICM42688.h>
#include <LPS25HB.h>
#include <NEC920.h>
#include <S25FL127S.h>
#include <S25FL512S.h>
#include <SPICREATE.h>

inline void aviApiSmoke() {
  static_assert(!std::is_copy_constructible_v<SPICREATE>);
  static_assert(!std::is_copy_constructible_v<CANCREATE>);
  static_assert(!std::is_copy_constructible_v<ICM42688>);
  static_assert(!std::is_copy_constructible_v<ICM20602>);
  static_assert(!std::is_copy_constructible_v<ICM20948>);
  static_assert(!std::is_copy_constructible_v<LPS25HB>);
  static_assert(!std::is_copy_constructible_v<S25FL127S>);

  // 不正GPIOと未初期化状態だけを使い、公開APIを実機I/Oなしでリンクする。
  SPICREATE spi;
  SPICREATE::Config spi_config{};
  spi_config.sck = -1;
  spi_config.transaction_timeout_ms = 1;
  (void)spi.begin(spi_config);
  (void)spi.begin(SPI2_HOST, -1, 13, 11);
  (void)spi.deviceCount();
  (void)spi.end();

  CANCREATE can;
  CANCREATE::Config can_config{};
  CANCREATE::Frame can_frame{};
  CANCREATE::Status can_status{};
  uint8_t can_data[]{0x73, 0x01};
  std::size_t available_count{};
  can_config.bitrate = CANCREATE::Bitrate::kbps500;
  (void)can.begin(can_config);
  (void)can.begin(GPIO_NUM_NC, GPIO_NUM_NC, CANCREATE::Bitrate::kbps500);
  (void)can.write(0x100, uint8_t{'s'}, 0);
  (void)can.write(0x100, can_data, 0);
  (void)can.write(can_frame, 0);
  (void)can.read(can_frame, 0);
  (void)can.available();
  (void)can.available(available_count);
  (void)can.getStatus(can_status);
  (void)can.recover(0);
  (void)can.initialized();
  (void)can.end();

  ICM42688 icm42688;
  ICM42688::Config icm42688_config{};
  ICM42688::Data icm42688_data{};
  ICM42688::RawData icm42688_raw{};
  ICM42688::Status icm42688_status{};
  uint8_t identity{};
  (void)icm42688.begin(spi, -1, icm42688_config);
  (void)icm42688.begin(spi, -1);
  (void)icm42688.whoAmI(identity);
  (void)icm42688.getStatus(icm42688_status);
  (void)icm42688.available();
  (void)icm42688.waitDataReady(0);
  (void)icm42688.readRaw(icm42688_raw);
  (void)icm42688.read(icm42688_data);
  (void)icm42688.initialized();
  (void)icm42688.end();

  ICM20948 icm20948;
  ICM20948::Config icm20948_config{};
  ICM20948::Data icm20948_data{};
  ICM20948::RawData icm20948_raw{};
  ICM20948::Status icm20948_status{};
  (void)icm20948.begin(spi, -1, icm20948_config);
  (void)icm20948.begin(spi, -1);
  (void)icm20948.whoAmI(identity);
  (void)icm20948.getStatus(icm20948_status);
  (void)icm20948.available();
  (void)icm20948.readRaw(icm20948_raw);
  (void)icm20948.read(icm20948_data);
  (void)icm20948.initialized();
  (void)icm20948.end();

  LPS25HB pressure;
  LPS25HB::Config pressure_config{};
  LPS25HB::Data pressure_data{};
  LPS25HB::RawData pressure_raw{};
  LPS25HB::Status pressure_status{};
  (void)pressure.begin(spi, -1, pressure_config);
  (void)pressure.begin(spi, -1);
  (void)pressure.whoAmI(identity);
  (void)pressure.getStatus(pressure_status);
  (void)pressure.available();
  (void)pressure.readRaw(pressure_raw);
  (void)pressure.read(pressure_data);
  (void)pressure.initialized();
  (void)pressure.end();

  S25FL127S flash;
  S25FL127S::Config flash_config{};
  S25FL127S::JedecId jedec_id{};
  S25FL127S::Status typed_flash_status{};
  uint8_t flash_status{};
  uint8_t buffer[S25FL127S::kPageSize]{};
  (void)flash.begin(spi, -1, flash_config);
  (void)flash.begin(spi, -1);
  (void)flash.readJedecId(jedec_id);
  (void)flash.readStatus(flash_status);
  (void)flash.getStatus(typed_flash_status);
  (void)flash.eraseBlock(0, 0);
  (void)flash.eraseChip(0);
  (void)flash.write(0, buffer, sizeof(buffer), 0);
  (void)flash.read(0, buffer, sizeof(buffer));
  (void)flash.writeByte(0, 0, 0);
  (void)flash.readByte(0, flash_status);
  (void)flash.initialized();
  (void)flash.end();

  ICM20602 icm20602;
  ICM20602::Config icm20602_config{};
  ICM20602::RawData icm20602_raw{};
  ICM20602::Data icm20602_data{};
  ICM20602::Status icm20602_status{};
  (void)icm20602.begin(spi, -1, icm20602_config);
  (void)icm20602.begin(spi, -1);
  (void)icm20602.whoAmI(identity);
  (void)icm20602.available();
  (void)icm20602.getStatus(icm20602_status);
  (void)icm20602.readRaw(icm20602_raw);
  (void)icm20602.read(icm20602_data);
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
