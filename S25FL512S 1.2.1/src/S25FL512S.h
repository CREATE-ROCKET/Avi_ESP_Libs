// version: 1.2.1
#pragma once

#ifndef SPIFlash_H
#define SPIFlash_H
#include <SPICREATE.h> // 2.0.0
#include <Arduino.h>

using namespace arduino::esp32::spi::dma;

#define CMD_RDID 0x9f
#define CMD_READ 0x03
#define CMD_4READ 0x13
#define CMD_FAST_READ 0x0B
#define CMD_4FAST_READ 0x0C
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_P4E 0x20
#define CMD_P8E 0x40

#define CMD_BE 0x60
#define CMD_PP 0x02
#define CMD_4PP 0x12
#define CMD_RDSR 0x05

#define ADDRESS_LENGTH 32
// #define PAGE_LENGTH 512 // You can change this number to an aliquot part of 512.
#define PAGE_LENGTH 256

// SPI Flashの最大のアドレス (1回で1/2ページ書き込んでいる点に注意)
// (512 * 1024 * 1024 / 8 / 256 ページ * 256) * 2 = 524288 * 256
uint32_t SPI_FLASH_MAX_ADDRESS = 0x8000000;

// SPIFlashLatestAddressは書き込むアドレス。初期値は0x000
// 0x000はreboot対策のどこまでSPI Flashに書き込んだかを記録するページ
// setup()で初期値でも0x100にしている
uint32_t SPIFlashLatestAddress = 0x000;

uint8_t count = 1;
uint8_t flashRead[256];
uint8_t flashRead1[256];
uint8_t flashRead2[256];

class Flash
{
    int CS;
    int deviceHandle{-1};
    SPICREATE::SPICreate *flashSPI;

public:
    void begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq = 8000000);
    uint32_t checkAddress(uint32_t FlashAddress);
    uint32_t setFlashAddress();
    void erase();
    void write(uint32_t addr, uint8_t *tx);
    void read(uint32_t addr, uint8_t *rx);
};

void Flash::begin(SPICREATE::SPICreate *targetSPI, int cs, uint32_t freq)
{
    CS = cs;
    flashSPI = targetSPI;
    spi_device_interface_config_t if_cfg = {};

    // if_cfg.spics_io_num = cs;
    if_cfg.pre_cb = NULL;
    // if_cfg.post_cb = NULL;
    if_cfg.cs_ena_pretrans = 0;
    if_cfg.cs_ena_posttrans = 0;

    if_cfg.clock_speed_hz = freq;
    if_cfg.command_bits = 0;
    if_cfg.address_bits = 0;
    if_cfg.mode = SPI_MODE3;
    if_cfg.queue_size = 1;
    if_cfg.pre_cb = csReset;
    if_cfg.post_cb = csSet;

    deviceHandle = flashSPI->addDevice(&if_cfg, cs);
    uint8_t readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);

    while (readStatus != 0)
    {
        readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);
        delay(100);
    }
    delay(100);
    return;
}

uint32_t Flash::checkAddress(uint32_t FlashAddress)
{
    count++;
    // if (count >= 20) // これ以上行くとページの途中を読むことになる。
    // {
    //   Serial.printf("FlashAddress: %u\n", FlashAddress);
    //   return FlashAddress;
    // }
    if ((FlashAddress + 0x100) >= SPI_FLASH_MAX_ADDRESS)
    {
        // Serial.printf("FlashAddress: %u\n", FlashAddress);
        // Serial.printf("count: %u\n", count);
        return SPI_FLASH_MAX_ADDRESS;
    }
    Flash::read(FlashAddress, flashRead1);
    delay(1);
    Flash::read(FlashAddress + 0x100, flashRead2);
    delay(1);
    if (flashRead1[0] != 0xFF)
    {
        if (flashRead2[0] != 0xFF)
        {
            // ++
            // Serial.println("---");
            // uint32_t a = FlashAddress + SPI_FLASH_MAX_ADDRESS / pow(2, count);
            // Serial.printf("FlashAddress: %u\n", a);
            // Serial.printf("count: %u\n", count);
            return checkAddress(FlashAddress + SPI_FLASH_MAX_ADDRESS / pow(2, count));
        }
        else
        {
            // Serial.printf("FlashAddress: %u\n", FlashAddress);
            return FlashAddress;
        }
    }
    else
    {
        if (flashRead2[0] != 0xFF)
        {
            // Serial.printf("FlashAddress: %u\n", FlashAddress);
            // Serial.println("error");
            return 0;
        }
        else
        {
            // --
            // Serial.printf("FlashAddress: %u\n", FlashAddress);
            return checkAddress(FlashAddress - SPI_FLASH_MAX_ADDRESS / pow(2, count));
        }
    }
}

uint32_t Flash::setFlashAddress()
{
    while (SPIFlashLatestAddress <= 0x1000)
    {
        Flash::read(SPIFlashLatestAddress, flashRead);
        // Serial.printf("SPIFlashLatestAddress: %u\n", SPIFlashLatestAddress);
        // Serial.print(flashRead[0]);
        delay(1);
        // Serial.print("\n");
        if (flashRead[0] == 0xFF)
        {
            Serial.println("255");
            return SPIFlashLatestAddress;
        }
        SPIFlashLatestAddress += 0x100;
        if (SPIFlashLatestAddress >= SPI_FLASH_MAX_ADDRESS)
        {
            // Serial.printf("SPIFlashLatestAddress: %u\n", SPIFlashLatestAddress);
            return SPIFlashLatestAddress;
        }
    }
    SPIFlashLatestAddress = checkAddress(SPI_FLASH_MAX_ADDRESS / 2);
    return SPIFlashLatestAddress;
}

void Flash::erase()
{
    Serial.println("start erase");
    if (flashSPI == NULL)
    {
        return;
    }

    flashSPI->sendCmd(CMD_WREN, deviceHandle);
    flashSPI->sendCmd(CMD_BE, deviceHandle);
    uint8_t readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);
    while (readStatus != 0)
    {
        readStatus = flashSPI->readByte(CMD_RDSR, deviceHandle);
        Serial.print(",");
        delay(100);
    }
    Serial.println("Bulk Erased");
    return;
}
void Flash::write(uint32_t addr, uint8_t *tx)
{
    flashSPI->sendCmd(CMD_WREN, deviceHandle);
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (PAGE_LENGTH)*8;
    comm.cmd = CMD_4PP;
    comm.addr = addr;
    comm.tx_buffer = tx;
    comm.user = (void *)&CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    spi_transaction.address_bits = ADDRESS_LENGTH;

    flashSPI->transmit((spi_transaction_t *)&spi_transaction, deviceHandle);
    return;
}
void Flash::read(uint32_t addr, uint8_t *rx)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    comm.length = (PAGE_LENGTH)*8;
    comm.cmd = CMD_4READ;
    comm.addr = addr;
    comm.tx_buffer = NULL;
    comm.rx_buffer = rx;
    comm.user = (void *)CS;

    spi_transaction_ext_t spi_transaction = {};
    spi_transaction.base = comm;
    spi_transaction.command_bits = 8;
    spi_transaction.address_bits = ADDRESS_LENGTH;
    flashSPI->transmit((spi_transaction_t *)&spi_transaction, deviceHandle);
}

#endif