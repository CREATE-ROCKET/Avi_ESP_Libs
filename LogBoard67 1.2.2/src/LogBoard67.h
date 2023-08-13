// version: 1.2.2
#pragma once

#ifndef LogBoard67_H
#define LogBoard67_H
#include <Arduino.h>
#include <SPICREATE.h>  // 2.0.0
#include <S25FL512S.h>  // 1.2.1
#include <H3LIS331.h>   // 1.2.0
#include <ICM20948.h>   // 2.0.0
#include <LPS25HB.h>    // 1.0.0
#include <Log67Timer.h> // 1.0.0

// センサのクラス
H3LIS331 H3lis331;
ICM icm20948;
LPS Lps25;
Flash flash1;

// Timerクラスのインスタンス化
Log67Timer timer;

class LogBoard67
{
private:
    // SPI_FlashBuffは送る配列
    uint8_t SPI_FlashBuff[256] = {};

    // CountSPIFlashDataSetExistInBuffは列
    int CountSPIFlashDataSetExistInBuff = 0;

    // 時間
    unsigned long Record_time;

    // 気圧の回数の測定(5回に1回)
    uint8_t count_lps = 0;

public:
    void RoutineWork();
};

void LogBoard67::RoutineWork()
{
    if (SPIFlashLatestAddress >= SPI_FLASH_MAX_ADDRESS)
    {
        Serial.printf("SPIFlashLatestAddress: %u\n", SPIFlashLatestAddress);
        // Serial2.write("SPI Flash is full");
        // Serial2.write("Started At: ");
        // Serial2.write(timer.start_time);
        // Serial2.write("Now: ");
        // Serial2.write(timer.Gettime_record());
        return;
    }
    // Serial.println("Running");
    if (timer.start_flag)
    {
        timer.start_time = micros();
        timer.start_flag = false;
    }
    Record_time = timer.Gettime_record();
    // From SPI, Get data is tx
    int16_t H3lisReceiveData[3] = {};
    uint8_t H3lis_rx_buf[6] = {};
    int16_t Icm20948ReceiveData[6] = {};
    uint8_t Icm20948_rx_buf[12] = {};
    uint8_t lps_rx[3] = {};
    // CountSPIFlashDataSetExistInBuffは列。indexは行。
    // 時間をとる
    for (int index = 0; index < 4; index++)
    {
        SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = 0xFF & (Record_time >> (8 * index));
    }

    // 加速度をとる
    H3lis331.Get2(H3lisReceiveData, H3lis_rx_buf);
    icm20948.Get(Icm20948ReceiveData, Icm20948_rx_buf);
    for (int index = 4; index < 10; index++)
    {
        SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = H3lis_rx_buf[index - 4];
    }

    // ICM20948の加速度をとる
    for (int index = 10; index < 16; index++)
    {
        SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = Icm20948_rx_buf[index - 10];
    }

    // ICM20948の角速度をとる
    for (int index = 16; index < 22; index++)
    {
        SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = Icm20948_rx_buf[index - 10];
    }

    // ICM20948の地磁気をとる
    // for (int index = 22; index < 28; index++)
    // {
    //   SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = Icm20948_rx_buf[index - 10];
    // }

    // LPSの気圧をとる
    if (count_lps % 20 == 0)
    {
        Lps25.Get(lps_rx);
        for (int index = 28; index < 31; index++)
        {
            SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = lps_rx[index - 28];
            count_lps = 0;
        }
    }

    count_lps++;
    CountSPIFlashDataSetExistInBuff++;

    // 8個のデータが溜まったらSPIFlashに書き込む
    if (CountSPIFlashDataSetExistInBuff >= 8)
    {
        // データの書き込み
        flash1.write(SPIFlashLatestAddress, SPI_FlashBuff);
        // アドレスの更新
        SPIFlashLatestAddress += 0x100;
        // 列の番号の初期化
        CountSPIFlashDataSetExistInBuff = 0;
    }
}

#endif