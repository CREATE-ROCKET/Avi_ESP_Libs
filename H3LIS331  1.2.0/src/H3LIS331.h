// version: 1.2.0
#pragma once

#ifndef H3LIS331_H
#define H3LIS331_H
#include <SPICREATE.h> // 2.0.0
#include <Arduino.h>

#define H3LIS331_Data_Address 0x28
#define H3LIS331_WhoAmI_Address 0x0F

#define H3LIS331_CTRL_REG1_Address 0x20 // CTRL_REG1 Power Mode & Output Data Rate
#define H3LIS331_CTRL_REG2_Address 0x21 // CTRL_REG2 (Normally they do not have to be changed)
#define H3LIS331_CTRL_REG3_Address 0x22 // CTRL_REG3 Interrupt
#define H3LIS331_CTRL_REG4_Address 0x23 // CTRL_REG4 ACC_CONFIG
#define H3LIS331_CTRL_REG5_Address 0x24 // CTRL_REG5 SleepToWake

#define H3LIS331_STATUS_REG_Address 0x27 // STATUS_REG Overrun

#define H3LIS331_CTRL_REG1 0b00111111 // CTRL_REG1 Normal Mode & OutPut Data Rate 1000Hz ????????????
// #define H3LIS331_CTRL_REG1 0b0001111       // CTRL_REG1 default value
#define H3LIS331_CTRL_REG2 0b00000000      // CTRL_REG2 (Normally they do not have to be changed)
#define H3LIS331_CTRL_REG3 0b00000000      // CTRL_REG3 Interrupt Active High
#define H3LIS331_CTRL_REG4_400G 0b00110000 // CTRL_REG4 400G
#define H3LIS331_CTRL_REG4_200G 0b00010000 // CTRL_REG4 200G
#define H3LIS331_CTRL_REG4_100G 0b00000000 // CTRL_REG4 100G
#define H3LIS331_CTRL_REG5 0b00000000      // CTRL_REG5 SleepToWake_OFF
#define H3LIS331_STATUS_REG 0b11111111     // STATUS_REG Overrun Available

class H3LIS331
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *H3LIS331SPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint8_t WhoImI();
    uint8_t WhoAmI();
    void Get(int16_t *rx);
    void Get2(int16_t *rx, uint8_t *rx_buf);
};

void H3LIS331::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{
    CS = cs;
    H3LIS331SPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = SPI_MODE0;
    if_cfg.queue_size = 1;

    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = H3LIS331SPI->addDevice(&if_cfg, cs);

    // 1kHzにする
    H3LIS331SPI->setReg(H3LIS331_CTRL_REG1_Address, H3LIS331_CTRL_REG1, deviceHandle);
    H3LIS331SPI->setReg(H3LIS331_CTRL_REG2_Address, H3LIS331_CTRL_REG2, deviceHandle);
    H3LIS331SPI->setReg(H3LIS331_CTRL_REG3_Address, H3LIS331_CTRL_REG3, deviceHandle);
    // Init
    H3LIS331SPI->setReg(H3LIS331_CTRL_REG4_Address, H3LIS331_CTRL_REG4_400G, deviceHandle);
    // SleepAndWakeをOFFにする
    H3LIS331SPI->setReg(H3LIS331_CTRL_REG5_Address, H3LIS331_CTRL_REG5, deviceHandle);
    H3LIS331SPI->setReg(H3LIS331_STATUS_REG_Address, H3LIS331_STATUS_REG, deviceHandle);
    return;
}
uint8_t H3LIS331::WhoImI()
{
    return H3LIS331::WhoAmI();
}
uint8_t H3LIS331::WhoAmI()
{
    return H3LIS331SPI->readByte(H3LIS331_WhoAmI_Address | 0x80, deviceHandle);
}
void H3LIS331::Get(int16_t *rx)
{
    uint8_t rx_buf[6];
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (6) * 8;

    uint16_t register_address = H3LIS331_Data_Address;
    register_address |= 0x40;
    comm.cmd = register_address | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    H3LIS331SPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);
    rx[0] = rx_buf[0];
    rx[0] |= ((uint16_t)rx_buf[1]) << 8;
    rx[1] = rx_buf[2];
    rx[1] |= ((uint16_t)rx_buf[3]) << 8;
    rx[2] = rx_buf[4];
    rx[2] |= ((uint16_t)rx_buf[5]) << 8;
    return;
}
void H3LIS331::Get2(int16_t *rx, uint8_t *rx_buf)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (6) * 8;

    uint16_t register_address = H3LIS331_Data_Address;
    register_address |= 0x40;
    comm.cmd = register_address | 0x80;

    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    H3LIS331SPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);
    rx[0] = rx_buf[0];
    rx[0] |= ((uint16_t)rx_buf[1]) << 8;
    rx[1] = rx_buf[2];
    rx[1] |= ((uint16_t)rx_buf[3]) << 8;
    rx[2] = rx_buf[4];
    rx[2] |= ((uint16_t)rx_buf[5]) << 8;
    return;
}
#endif