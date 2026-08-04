#include <Arduino.h>
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
void setup() {
  SPICREATE spi;
  CANCREATE can;
  ICM42688 a;
  ICM20948 b;
  ICM20602 c;
  H3LIS331 d;
  LPS25HB e;
  S25FL127S f;
  S25FL512S g;
  NEC920 h;
  (void)spi;
  (void)can;
  (void)a;
  (void)b;
  (void)c;
  (void)d;
  (void)e;
  (void)f;
  (void)g;
  (void)h;
}
void loop() {}
