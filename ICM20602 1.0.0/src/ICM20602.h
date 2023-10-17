#pragma once

#ifndef ICM_H
#define ICM_H
#include <SPICREATE.h>
#include <Arduino.h>

#define ICM_CONFIG 0x1A
#define ICM_PWR_MGMT_1 0x6B
#define ICM_GYRO_CONFIG 0x1B
#define ICM_ACC_CONFIG 0x1C
#define ICM_16G 0b00011000
#define ICM_8G 0b00010000
#define ICM_4G 0b00001000
#define ICM_2G 0b00000000
#define ICM_2000dps 0b00011000
#define ICM_1000dps 0b00010000
#define ICM_500dps 0b00001000
#define ICM_250dps 0b00000000
#define ICM_WhoAmI_Adress 0x75
#define ICM_Data_Adress 0x3B
#define ICM_I2C_IF 0x70

class ICM
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *ICMSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoAmI(); // Return 0x12
    void Get(int16_t *rx);
    void Get(int16_t *rx, uint8_t *rx_raw);
    float AccelNorm = 0.;
};

IRAM_ATTR void ICM::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
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

    if_cfg.mode = SPI_MODE3; // 0 or 3
    if_cfg.queue_size = 1;
    if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = ICMSPI->addDevice(&if_cfg, cs);
    ICMSPI->setReg(ICM_I2C_IF, 0b01000000, deviceHandle);
    ICMSPI->setReg(ICM_CONFIG, 0x00, deviceHandle);
    ICMSPI->setReg(ICM_PWR_MGMT_1, 0x01, deviceHandle);
    ICMSPI->setReg(ICM_ACC_CONFIG, ICM_16G, deviceHandle);
    ICMSPI->setReg(ICM_GYRO_CONFIG, ICM_2000dps, deviceHandle);
    return;
}
IRAM_ATTR uint8_t ICM::WhoAmI()
{
    return ICMSPI->readByte(0x80 | ICM_WhoAmI_Adress, deviceHandle);
}

IRAM_ATTR void ICM::Get(int16_t *rx)
{
    uint8_t rx_raw[14];
    Get(rx, rx_raw);
}

IRAM_ATTR void ICM::Get(int16_t *rx, uint8_t *rx_raw)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (14) * 8;
    comm.cmd = ICM_Data_Adress | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_raw;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    ICMSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);

    rx[0] = (int16_t)(rx_raw[0] << 8 | rx_raw[1]);
    rx[1] = (int16_t)(rx_raw[2] << 8 | rx_raw[3]);
    rx[2] = (int16_t)(rx_raw[4] << 8 | rx_raw[5]);
    rx[3] = (int16_t)(rx_raw[8] << 8 | rx_raw[9]);
    rx[4] = (int16_t)(rx_raw[10] << 8 | rx_raw[11]);
    rx[5] = (int16_t)(rx_raw[12] << 8 | rx_raw[13]);

    float Accel_Buf = rx[0] * rx[0] + rx[1] * rx[1] + rx[2] * rx[2];
    AccelNorm = (float)(sqrt((float)(Accel_Buf)) * 16. / 32768.);
    return;
}
#endif
