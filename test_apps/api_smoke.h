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
  std::size_t available_count{};
  (void)can.begin(can_config);
  (void)can.begin(GPIO_NUM_NC, GPIO_NUM_NC);
  (void)can.write(can_frame, 0);
  (void)can.read(can_frame, 0);
  (void)can.available(available_count);
  (void)can.getStatus(can_status);
  (void)can.recover(0);
  (void)can.initialized();
  (void)can.end();

  ICM42688 icm42688;
  ICM42688::Config icm42688_config{};
  ICM42688::Data icm42688_data{};
  ICM42688::Status icm42688_status{};
  uint8_t identity{};
  (void)icm42688.begin(spi, -1, icm42688_config);
  (void)icm42688.begin(spi, -1);
  (void)icm42688.whoAmI(identity);
  (void)icm42688.getStatus(icm42688_status);
  (void)icm42688.waitDataReady(0);
  (void)icm42688.get(icm42688_data);
  (void)icm42688.initialized();
  (void)icm42688.end();

  ICM20948 icm20948;
  ICM20948::Config icm20948_config{};
  ICM20948::Data icm20948_data{};
  ICM20948::Status icm20948_status{};
  (void)icm20948.begin(spi, -1, icm20948_config);
  (void)icm20948.begin(spi, -1);
  (void)icm20948.whoAmI(identity);
  (void)icm20948.getStatus(icm20948_status);
  (void)icm20948.get(icm20948_data);
  (void)icm20948.initialized();
  (void)icm20948.end();

  LPS25HB pressure;
  LPS25HB::Config pressure_config{};
  LPS25HB::Data pressure_data{};
  LPS25HB::Status pressure_status{};
  (void)pressure.begin(spi, -1, pressure_config);
  (void)pressure.begin(spi, -1);
  (void)pressure.whoAmI(identity);
  (void)pressure.getStatus(pressure_status);
  (void)pressure.get(pressure_data);
  (void)pressure.initialized();
  (void)pressure.end();

  S25FL127S flash;
  S25FL127S::Config flash_config{};
  S25FL127S::JedecId jedec_id{};
  uint8_t flash_status{};
  uint8_t buffer[S25FL127S::kPageSize]{};
  (void)flash.begin(spi, -1, flash_config);
  (void)flash.begin(spi, -1);
  (void)flash.readJedecId(jedec_id);
  (void)flash.readStatus(flash_status);
  (void)flash.erase(0);
  (void)flash.write(0, buffer, sizeof(buffer), 0);
  (void)flash.read(0, buffer, sizeof(buffer));
  (void)flash.initialized();
  (void)flash.end();

  ICM20602 icm20602;
  H3LIS331 h3lis331;
  S25FL512S flash512;
  NEC920 radio;
  (void)icm20602;
  (void)h3lis331;
  (void)flash512;
  (void)radio;
}
