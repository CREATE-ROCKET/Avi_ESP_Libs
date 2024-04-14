// version: 1.0.0
#pragma once

#ifndef ICM_H
#define ICM_H
#include <SPICREATE.h> // 2.0.0
#include <Arduino.h>

#define POWER_MANAGEMENT 0x4E
#define WHO_AM_I_Address 0x75
#define ICM_Data_Adress 0x1F

class ICM
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *ICMSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
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

    ICMSPI->setReg(POWER_MANAGEMENT, 0x0F, deviceHandle);
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
    return;
}
#endif
