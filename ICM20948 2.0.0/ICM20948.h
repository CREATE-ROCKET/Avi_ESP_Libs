// version: 2.0.0
#pragma once

#ifndef ICM_H
#define ICM_H
#include <SPICREATE.h> // 2.0.0
#include <Arduino.h>

#define ICM_Data_Adress 0x2D   // BANK0
#define ICM_GYRO_CONFIG 0x01   // BANK2
#define ICM_WhoAmI_Adress 0x00 // BANK0 default0xEA
#define ICM_ACC_CONFIG 0x14    // BANK2
#define ICM_REG_BANK 0x7F      // default BANK0
#define ICM_PWR_MGMT 0x06      // BANK0
#define ICM_USER_CTRL 0x03     // BANK0
#define ICM_16G 0b00000110
#define ICM_8G 0b00000100
#define ICM_4G 0b00000010
#define ICM_2G 0b00000000
#define ICM_2000dps 0b00000110
#define ICM_1000dps 0b00000010
#define ICM_500dps 0b00000100
#define ICM_250dps 0b00000000
#define ICM_USER_BANK0 0b00000000
#define ICM_USER_BANK1 0b00010000
#define ICM_USER_BANK2 0b00100000
#define ICM_USER_BANK3 0b00110000

// #define ICM_2500deg 0x18

class ICM
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *ICMSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoAmI();
    uint8_t UserBank();
    void Get(int16_t *rx, uint8_t *rx_buf);
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

    ICMSPI->setReg(ICM_USER_CTRL, 0x10, deviceHandle);
    ICMSPI->setReg(ICM_PWR_MGMT, 0x01, deviceHandle); // turn off sleep mode
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK2, deviceHandle);
    ICMSPI->setReg(ICM_ACC_CONFIG, ICM_16G, deviceHandle);
    ICMSPI->setReg(ICM_GYRO_CONFIG, ICM_2000dps, deviceHandle);
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    return;
}
uint8_t ICM::WhoAmI()
{
    return ICMSPI->readByte(0x80 | 0x06, deviceHandle);
}

void ICM::Get(int16_t *rx, uint8_t *rx_buf)
{
    // uint8_t rx_buf[12];
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
