// version: 2.0.0
#pragma once

#ifndef SPICREATE_Master
#define SPICREATE_Master

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_master.h>
#include <deque>

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define IS_S3 1
#elif CONFIG_IDF_TARGET_ESP32
#define IS_S3 0
#else
#error "No supported board specified!!!"
#endif

void csSet(spi_transaction_t *t);
void csReset(spi_transaction_t *t);
namespace arduino
{
    namespace esp32
    {
        namespace spi
        {
            namespace dma
            {

                class SPICreate
                {
                    spi_bus_config_t bus_cfg = {};
                    spi_device_handle_t handle[3];
                    int deviceNum{0};
#if IS_S3
                    spi_host_device_t host{SPI2_HOST};
#else
                    spi_host_device_t host{HSPI_HOST};
#endif
                    uint8_t mode{SPI_MODE3};       // must be 1 or 3
                    int max_size{SPI_MAX_DMA_LEN}; // default size
                    uint32_t frequency{SPI_MASTER_FREQ_8M};

                    std::deque<spi_transaction_t> transactions;
                    int queue_size{1};

                public:
#if !(IS_S3)
                    bool begin(
                        uint8_t spi_bus = HSPI,
                        int8_t sck = -1,
                        int8_t miso = -1,
                        int8_t mosi = -1,
                        uint32_t f = SPI_MASTER_FREQ_8M);
#endif
                    bool begin(
                        spi_host_device_t host_in = SPI2_HOST,
                        int8_t sck = -1,
                        int8_t miso = -1,
                        int8_t mosi = -1,
                        uint32_t f = SPI_MASTER_FREQ_8M);

                    bool end();

                    int addDevice(spi_device_interface_config_t *if_cfg, int cs);
                    bool rmDevice(int deviceHandle);

                    uint8_t readByte(uint8_t addr, int deviceHandle);
                    void sendCmd(uint8_t cmd, int deviceHandle);
                    void setReg(uint8_t addr, uint8_t data, int deviceHandle);

                    void transmit(uint8_t *tx, int size, int deviceHandle);
                    void transmit(uint8_t *tx, uint8_t *rx, int size, int deviceHandle);
                    void transmit(spi_transaction_t *transaction, int deviceHandle);

                    void pollTransmit(spi_transaction_t *transaction, int deviceHandle);
                };
            } // dma
        } // spi
    } // esp32
} // arduino

#ifndef SPICREATE_BEGIN
#define SPICREATE_BEGIN       \
    namespace arduino         \
    {                         \
        namespace esp32       \
        {                     \
            namespace spi     \
            {                 \
                namespace dma \
                {
#endif
#ifndef SPICREATE_END
#define SPICREATE_END \
    }                 \
    }                 \
    }                 \
    }
#endif
namespace SPICREATE = arduino::esp32::spi::dma;
#endif