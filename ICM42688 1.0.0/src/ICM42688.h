// version: 1.0.0
//todo calib
#pragma once

#ifndef ICM_H
#define ICM_H
#include <SPICREATE.h> // 2.0.0
#include <Arduino.h>

#define POWER_MANAGEMENT 0x4E
#define WHO_AM_I_Address 0x75
#define ICM_Data_Adress 0x1F
#define GYRO_ACCEL_CONFIG0 0x52
#define REG_BANK_SEL 0x76
#define OFFSET_USER0 0x77
#define TMST_VALUE 0x62
#define TMST_CONFIG 0x54
#define FIFO_CONFIG 0x5F

class ICM
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *ICMSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    void begincalib(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoAmI();
    uint8_t UserBank();
    void Get(int16_t *rx);
};

void ICM::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{
    CS = cs;
    ICMSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = SPI_MODE0; // 0 or 3
    if_cfg.queue_size = 1;

    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = ICMSPI->addDevice(&if_cfg, cs);
    delay(1);
    ICMSPI->setReg(REG_BANK_SEL, 0x04, deviceHandle);//USER BANK 1 to 4
    delay(1);
    ICMSPI->setReg(0x77, 0b11110111, deviceHandle);//キャリブレーション(個体値ごとの調整が必要)
    delay(1);
    ICMSPI->setReg(0x78, 0b00001111, deviceHandle);
    delay(1);
    ICMSPI->setReg(0x79, 0b00010000, deviceHandle);
    delay(1);
    ICMSPI->setReg(0x7A, 0b00000100, deviceHandle);
    delay(1);
    ICMSPI->setReg(0x7B, 0b00000000, deviceHandle); 
    delay(1);
    ICMSPI->setReg(REG_BANK_SEL, 0x00, deviceHandle);//USER BANK 1 to 4
    delay(1);
    ICMSPI->setReg(GYRO_ACCEL_CONFIG0, 0b00010101, deviceHandle);//60.8hz cutoff, 6.6ms delay 
    delay(1);
    ICMSPI->setReg(FIFO_CONFIG, 0b01111011, deviceHandle);//FIFO confi
    delay(1);
    ICMSPI->setReg(TMST_CONFIG, 0b00110001, deviceHandle);//timestanp
    delay(1);
    ICMSPI->setReg(POWER_MANAGEMENT, 0x0F, deviceHandle);
    delay(1);

    return;
}


uint8_t ICM::WhoAmI()
{
    return ICMSPI->readByte(0x80 | WHO_AM_I_Address, deviceHandle);
}

/**
 * @fn
 * ICMから加速度、角速度を取得
 */
void ICM::Get(int16_t *rx)
{
    uint8_t rx_buf[12];
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (12) * 8;
    comm.cmd = ICM_Data_Adress | 0x80;
    // comm.cmd = 0x77 | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    ICMSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);

    rx[0] = (rx_buf[0] << 8 | rx_buf[1]);
    rx[1] = (rx_buf[2] << 8 | rx_buf[3]);
    rx[2] = (rx_buf[4] << 8 | rx_buf[5]);
    rx[3] = (rx_buf[6] << 8 | rx_buf[7]);
    rx[4] = (rx_buf[8] << 8 | rx_buf[9]);
    rx[5] = (rx_buf[10] << 8 | rx_buf[11]);

    ICMSPI->setReg(REG_BANK_SEL, 0x01, deviceHandle);//USER BANK 1 to 4
    spi_transaction_t comm2 = {};
    comm2.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm2.length = (3) * 8;
    comm2.cmd = TMST_VALUE | 0x80;
    // comm.cmd = 0x77 | 0x80;

    comm2.tx_buffer = NULL;
    comm2.rx_buffer = rx_buf;
    comm2.user = (void *)CS;

    spi_transaction_ext_t spi_transaction2 = {};
    spi_transaction2.base = comm2;
    spi_transaction2.command_bits = 8;
    ICMSPI->pollTransmit((spi_transaction_t *)&spi_transaction2, deviceHandle);
    rx[6] = (rx_buf[0] << 8 | rx_buf[1]);
    rx[7] = (rx_buf[2] << 8 | rx_buf[3]);   
    ICMSPI->setReg(REG_BANK_SEL, 0x00, deviceHandle);//USER BANK 1 to 4
    return;
}

void ICM::begincalib(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)//キャリブレーション用に低FSRに設定
{
    CS = cs;
    ICMSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = SPI_MODE0; // 0 or 3
    if_cfg.queue_size = 1;

    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = ICMSPI->addDevice(&if_cfg, cs);
    ICMSPI->setReg(REG_BANK_SEL, 0x00, deviceHandle);//USER BANK 1 to 4
    delay(1);
    ICMSPI->setReg(GYRO_ACCEL_CONFIG0, 0b00010101, deviceHandle);//60.8hz cutoff, 6.6ms delay 
    delay(1);
    // ICMSPI->setReg(REG_BANK_SEL, 0x04, deviceHandle);//USER BANK 1 to 4
    delay(1);
    ICMSPI->setReg(0x5F, 0b01111011, deviceHandle);//FIFO confi
    delay(1);
    ICMSPI->setReg(0x54, 0b00110001, deviceHandle);//timestanp
    delay(1);
    ICMSPI->setReg(0x4F, 0b11100110, deviceHandle);
    delay(1);    
    ICMSPI->setReg(POWER_MANAGEMENT, 0x0F, deviceHandle);
    delay(1);

    return;
}
#endif
