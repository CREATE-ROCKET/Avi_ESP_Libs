// version: 2.0.0
#pragma once

#ifndef ICM_H
#define ICM_H
#include <Arduino.h>
#include <SPICREATE.h>  // 2.0.0

#define ICM_Data_Adress 0x2D    // BANK0
#define ICM_GYRO_CONFIG 0x01    // BANK2
#define ICM_WhoAmI_Adress 0x00  // BANK0 default0xEA
#define ICM_ACC_CONFIG 0x14     // BANK2
#define ICM_REG_BANK 0x7F       // default BANK0
#define ICM_PWR_MGMT 0x06       // BANK0
#define ICM_USER_CTRL 0x03      // BANK0
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

#define ICM_INT_PIN_CFG 0x0F      // BANK0
#define ICM_LP_CONFIG 0x05        // BANK0
#define ICM_I2C_MST_STATUS 0x17   // BANK0
#define ICM_I2C_MST_CTRL 0x01     // BANK3
#define ICM_MagData_Address 0x3B  // BANK0
#define ICM_I2C_SLV0_ADDR 0x03    // BANK3
#define ICM_I2C_SLV0_REG 0x04     // BANK3
#define ICM_I2C_SLV0_CTRL 0x05    // BANK3
#define ICM_I2C_SLV0_DO 0x06      // BANK3
#define ICM_I2C_SLV1_ADDR 0x07    // BANK3
#define ICM_I2C_SLV1_REG 0x08     // BANK3
#define ICM_I2C_SLV1_CTRL 0x09    // BANK3
#define ICM_I2C_SLV1_DO 0x0A      // BANK3
#define ICM_I2C_SLV2_ADDR 0x0B    // BANK3
#define ICM_I2C_SLV2_REG 0x0C     // BANK3
#define ICM_I2C_SLV2_CTRL 0x0D    // BANK3
#define ICM_I2C_SLV2_DO 0x0E      // BANK3
#define ICM_I2C_SLV3_ADDR 0x0F    // BANK3
#define ICM_I2C_SLV3_REG 0x10     // BANK3
#define ICM_I2C_SLV3_CTRL 0x11    // BANK3
#define ICM_I2C_SLV3_DO 0x12      // BANK3
#define ICM_I2C_SLV4_ADDR 0x13    // BANK3
#define ICM_I2C_SLV4_REG 0x14     // BANK3
#define ICM_I2C_SLV4_CTRL 0x15    // BANK3
#define ICM_I2C_SLV4_DO 0x16      // BANK3
#define ICM_I2C_SLV4_DI 0x17      // BANK3

#define AK09916_I2C_address 0x0C
// ↓AK09916 registers

#define AK09916_REG_WIA1 0x00
#define AK09916_REG_WIA2 0x01
#define AK09916_REG_ST1 0x10
#define AK09916_REG_CNTL2 0x31
#define AK09916_REG_CNTL3 0x32

class ICM {
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *ICMSPI;

    uint8_t readMag(uint8_t reg);
    void writeMag(uint8_t reg, uint8_t data);
    void ICM_20948_i2c_controller_periph4_txn(
        uint8_t addr, uint8_t reg, uint8_t *data,
        bool Rw);  // 1: Read 0: Write  len is always 1
    void ICM_20948_i2c_master_single_w(uint8_t addr, uint8_t reg, uint8_t data);
    uint8_t ICM_20948_i2c_master_single_r(uint8_t addr, uint8_t reg);
    void i2c_master_reset();
    void resetMag();
    void i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr,
                                          uint8_t reg, uint8_t len, bool Rw,
                                          bool enable, bool data_only, bool grp,
                                          bool swap, uint8_t dataOut);

    void i2c_master_enable();

   public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs,
               uint32_t freq = 8000000);
    uint8_t WhoAmI();
    uint8_t UserBank();
    void Get(int16_t *rx, uint8_t *rx_buf);
    void GetMag(int16_t *rx);
    void magWhoAmI(uint8_t *who1, uint8_t *who2);  // shoud be 1:0x48, 2:0x09
    void startupMagnetometer();
};

void ICM::ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg,
                                               uint8_t *data, bool Rw) {
    addr = (((Rw) ? 0x80 : 0x00) | addr);
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
    ICMSPI->setReg(ICM_I2C_SLV4_ADDR, addr, deviceHandle);
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
    ICMSPI->setReg(ICM_I2C_SLV4_REG, reg, deviceHandle);

    uint8_t i2c_mst_status;
    bool txn_failed = false;

    if (!Rw) {
        ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
        ICMSPI->setReg(ICM_I2C_SLV4_DO, *data, deviceHandle);
    }
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
    ICMSPI->setReg(ICM_I2C_SLV4_CTRL, 0b10000000, deviceHandle);
    delay(5);
    if (Rw) {
        ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
        *data = ICMSPI->readByte(ICM_I2C_SLV4_DI | 0x80, deviceHandle);
    }
    delay(1);
    return;
}
void ICM::ICM_20948_i2c_master_single_w(uint8_t addr, uint8_t reg,
                                        uint8_t data) {
    ICM_20948_i2c_controller_periph4_txn(addr, reg, &data, false);
    return;
}
uint8_t ICM::ICM_20948_i2c_master_single_r(uint8_t addr, uint8_t reg) {
    uint8_t data;
    ICM_20948_i2c_controller_periph4_txn(addr, reg, &data, true);
    return data;
}
uint8_t ICM::readMag(uint8_t reg) {
    uint8_t data = ICM_20948_i2c_master_single_r(AK09916_I2C_address, reg);
    return data;
}
void ICM::writeMag(uint8_t reg, uint8_t data) {
    ICM_20948_i2c_master_single_w(AK09916_I2C_address, reg, data);
    return;
}
void ICM::i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr,
                                           uint8_t reg, uint8_t len, bool Rw,
                                           bool enable, bool data_only,
                                           bool grp, bool swap,
                                           uint8_t dataOut) {
    uint8_t periph_addr_reg;
    uint8_t periph_reg_reg;
    uint8_t periph_ctrl_reg;
    uint8_t periph_do_reg;
    switch (peripheral) {
        case 0:
            periph_addr_reg = ICM_I2C_SLV0_ADDR;
            periph_reg_reg = ICM_I2C_SLV0_REG;
            periph_ctrl_reg = ICM_I2C_SLV0_CTRL;
            periph_do_reg = ICM_I2C_SLV0_DO;
            break;
        case 1:
            periph_addr_reg = ICM_I2C_SLV1_ADDR;
            periph_reg_reg = ICM_I2C_SLV1_REG;
            periph_ctrl_reg = ICM_I2C_SLV1_CTRL;
            periph_do_reg = ICM_I2C_SLV1_DO;
            break;
        case 2:
            periph_addr_reg = ICM_I2C_SLV2_ADDR;
            periph_reg_reg = ICM_I2C_SLV2_REG;
            periph_ctrl_reg = ICM_I2C_SLV2_CTRL;
            periph_do_reg = ICM_I2C_SLV2_DO;
            break;
        case 3:
            periph_addr_reg = ICM_I2C_SLV3_ADDR;
            periph_reg_reg = ICM_I2C_SLV3_REG;
            periph_ctrl_reg = ICM_I2C_SLV3_CTRL;
            periph_do_reg = ICM_I2C_SLV3_DO;
            break;

        default:
            return;
            break;
    }
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
    uint8_t address = addr;
    if (Rw) {
        address |= 0b10000000;
    }
    ICMSPI->setReg(periph_addr_reg, address, deviceHandle);
    if (!Rw) {
        ICMSPI->setReg(periph_do_reg, dataOut, deviceHandle);
    }
    ICMSPI->setReg(periph_reg_reg, reg, deviceHandle);
    ICMSPI->setReg(periph_ctrl_reg, 0x89,
                   deviceHandle);  //<-this value 0x89 is for only magnetrometer
    return;
}
void ICM::i2c_master_enable() {
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    uint8_t reg = ICMSPI->readByte(ICM_INT_PIN_CFG | 0x80, deviceHandle);
    reg &= 0b11111101;
    ICMSPI->setReg(ICM_INT_PIN_CFG, reg,
                   deviceHandle);  // disable I2C passthrough
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK3, deviceHandle);
    ICMSPI->setReg(ICM_I2C_MST_CTRL, 0x17, deviceHandle);
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    uint8_t ctrl = ICMSPI->readByte(ICM_USER_CTRL | 0x80, deviceHandle);
    ctrl |= 0b00100000;
    ICMSPI->setReg(ICM_USER_CTRL, ctrl, deviceHandle);
    return;
}
void ICM::i2c_master_reset() {
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    uint8_t ctrl = ICMSPI->readByte(ICM_USER_CTRL | 0x80, deviceHandle);
    ctrl |= 0b00000010;
    ICMSPI->setReg(ICM_USER_CTRL, ctrl, deviceHandle);
    return;
}
void ICM::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq) {
    CS = cs;
    ICMSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;

    if_cfg.mode = SPI_MODE0;  // 0 or 3
    if_cfg.queue_size = 1;

    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = ICMSPI->addDevice(&if_cfg, cs);

    ICMSPI->setReg(ICM_USER_CTRL, 0x10, deviceHandle);
    ICMSPI->setReg(ICM_PWR_MGMT, 0x01, deviceHandle);  // turn off sleep mode
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK2, deviceHandle);
    ICMSPI->setReg(ICM_ACC_CONFIG, ICM_16G, deviceHandle);
    ICMSPI->setReg(ICM_GYRO_CONFIG, ICM_2000dps, deviceHandle);
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    startupMagnetometer();
    delay(5);
    return;
}
uint8_t ICM::WhoAmI() {
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    return ICMSPI->readByte(0x80 | ICM_WhoAmI_Adress, deviceHandle);
}
void ICM::startupMagnetometer() {
    i2c_master_enable();
    resetMag();
    delay(10);
    uint8_t tries = 0;
    while (tries < 10) {
        tries++;
        uint8_t who1, who2;
        magWhoAmI(&who1, &who2);
        if (who1 == 0x48) {
            break;
        }
        i2c_master_reset();
        delay(10);
    }

    writeMag(AK09916_REG_CNTL2, 0x00);
    delay(1);
    i2cControllerConfigurePeripheral(0, AK09916_I2C_address, AK09916_REG_ST1, 9,
                                     true, true, false, false, false,
                                     (uint8_t)0U);
    writeMag(AK09916_REG_CNTL2, 0x08);
    delay(1);
    return;
}

void ICM::magWhoAmI(uint8_t *who1, uint8_t *who2) {
    *who1 = readMag(AK09916_REG_WIA1);
    delay(1);
    *who2 = readMag(AK09916_REG_WIA2);
    return;
}
void ICM::resetMag() {
    uint8_t SRST = 1;
    ICM_20948_i2c_master_single_w(AK09916_I2C_address, AK09916_REG_CNTL3, SRST);
    return;
}
void ICM::Get(int16_t *rx, uint8_t *rx_buf) {
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
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
void ICM::GetMag(int16_t *rx) {
    ICMSPI->setReg(ICM_REG_BANK, ICM_USER_BANK0, deviceHandle);
    uint8_t rx_buf[9];
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (9) * 8;
    comm.cmd = ICM_MagData_Address | 0x80;
    comm.tx_buffer = NULL;
    comm.rx_buffer = rx_buf;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    ICMSPI->pollTransmit((spi_transaction_t *)&spi_transaction, deviceHandle);

    rx[0] = ((rx_buf[2] << 8) | rx_buf[1] & 0xFF);
    rx[1] = ((rx_buf[4] << 8) | rx_buf[3] & 0xFF);
    rx[2] = ((rx_buf[6] << 8) | rx_buf[5] & 0xFF);
    return;
}
#endif
