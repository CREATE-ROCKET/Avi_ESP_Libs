// version: 1.0.0
#pragma once

#ifndef LPS_H
#define LPS_H
#include <SPICREATE.h> // 2.0.0
#include <Arduino.h>

#define LPS_Data_Adress_0 0x28
#define LPS_Data_Adress_1 0x29
#define LPS_Data_Adress_2 0x2A

#define LPS_WakeUp_Adress 0x20
#define LPS_WakeUp_Value 0xC0
#define LPS_Setting_Adress 0x21
#define LPS_Settig_Value 0x08
#define LPS_WhoAmI_Adress 0x0F

class LPS
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *LPSSPI;

public:
    uint32_t PlessureRaw;
    int Plessure;
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoAmI();
    void Get(uint8_t *rx);
    // (uint32_t)rx[2] << 16 | (uint32_t)rx[1] << 8 | (uint32_t)rx[0] means pressure
};

void LPS::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{
    CS = cs;
    LPSSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = SPI_MODE3;
    if_cfg.queue_size = 1;

    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = LPSSPI->addDevice(&if_cfg, cs);
    LPSSPI->setReg(LPS_Setting_Adress, LPS_Settig_Value, deviceHandle);
    LPSSPI->setReg(LPS_WakeUp_Adress, LPS_WakeUp_Value, deviceHandle);

    return;
}
uint8_t LPS::WhoAmI()
{
    return LPSSPI->readByte(LPS_WhoAmI_Adress | 0x80, deviceHandle);
    // registor 0x0F and you'll get 0d177 or 0xb1 or 0b10110001
}

void LPS::Get(uint8_t *rx)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (14) * 8;
    comm.cmd = LPS_Data_Adress_0 | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = NULL;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;

    rx[0] = LPSSPI->readByte(LPS_Data_Adress_0 | 0x80, deviceHandle);
    rx[1] = LPSSPI->readByte(LPS_Data_Adress_1 | 0x80, deviceHandle);
    rx[2] = LPSSPI->readByte(LPS_Data_Adress_2 | 0x80, deviceHandle);
    PlessureRaw = (uint32_t)rx[2] << 16 | (uint32_t)rx[1] << 8 | (uint32_t)rx[0];
    Plessure = (int)PlessureRaw * 100 / 4096;
    return;
}

#endif