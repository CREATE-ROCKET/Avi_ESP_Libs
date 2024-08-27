// version: 2.0.0
#include "SPICREATE.h" // 2.0.0
void csSet(spi_transaction_t *t)
{
    digitalWrite((int)t->user, HIGH);
    return;
}
void csReset(spi_transaction_t *t)
{
    digitalWrite((int)t->user, LOW);
    return;
}
SPICREATE_BEGIN

bool SPICreate::begin(uint8_t spi_bus, int8_t sck, int8_t miso, int8_t mosi, uint32_t f)
{

    frequency = f;
    if ((sck == -1) && (miso == -1) && (mosi == -1))
    {
        bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK : 14;
        bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
        bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
    }
    else
    {
        bus_cfg.sclk_io_num = sck;
        bus_cfg.miso_io_num = miso;
        bus_cfg.mosi_io_num = mosi;
    }

    bus_cfg.max_transfer_sz = max_size;

    if ((mode != SPI_MODE1) && (mode != SPI_MODE3))
    {
        mode = SPI_MODE3;
    }

    host = (spi_bus == HSPI) ? HSPI_HOST : VSPI_HOST;
    dma_chan = 1;
    if (spi_bus == VSPI)
    {
        dma_chan = 1;
    }
    if (spi_bus == HSPI)
    {
        dma_chan = 2;
    }
    esp_err_t e = spi_bus_initialize(host, &bus_cfg, dma_chan);
    if (e != ESP_OK)
    {
        // printf("[ERROR] SPI bus initialize failed : %d\n", e);
        return false;
    }

    return true;
}
bool SPICreate::end()
{
    esp_err_t e = spi_bus_free(host);
    if (e != ESP_OK)
    {
        // printf("[ERROR] SPI bus free failed : %d\n", e);
        return false;
    }

    return true;
}
int SPICreate::addDevice(spi_device_interface_config_t *if_cfg, int cs)
{
    deviceNum++;
    CSs[deviceNum] = cs;
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    if (deviceNum > 9)
    {
        return 0;
    }
    esp_err_t e = spi_bus_add_device(host, if_cfg, &handle[deviceNum]);
    if (e != ESP_OK)
    {
        return 0;
    }
    return deviceNum;
}

bool SPICreate::rmDevice(int deviceHandle)
{
    esp_err_t e = spi_bus_remove_device(handle[deviceHandle]);
    if (e != ESP_OK)
    {
        // printf("[ERROR] SPI bus remove device failed : %d\n", e);
        return false;
    }
    return true;
}
void SPICreate::sendCmd(uint8_t cmd, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_TXDATA;
    comm.length = 8;
    comm.tx_data[0] = cmd;
    comm.user = (void *)CSs[deviceHandle];
    pollTransmit(&comm, deviceHandle);
}
uint8_t SPICreate::readByte(uint8_t addr, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    comm.tx_data[0] = addr;
    comm.length = 16;
    comm.user = (void *)CSs[deviceHandle];
    pollTransmit(&comm, deviceHandle);
    return comm.rx_data[1];
}
void SPICreate::setReg(uint8_t addr, uint8_t data, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_TXDATA;
    comm.length = 16;
    comm.tx_data[0] = addr;
    comm.tx_data[1] = data;
    transmit(&comm, deviceHandle);
}
void SPICreate::transmit(uint8_t *tx, int size, int deviceHandle)
{
    transmit(tx, NULL, size, deviceHandle);
}
void SPICreate::transmit(uint8_t *tx, uint8_t *rx, int size, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.length = size * 2 * 8;
    comm.rxlength = size * 8;
    comm.tx_buffer = tx;
    comm.rx_buffer = rx;
    transmit(&comm, deviceHandle);
}

void SPICreate::transmit(spi_transaction_t *transaction, int deviceHandle)
{
    digitalWrite(CSs[deviceHandle], LOW);
    esp_err_t e = spi_device_transmit(handle[deviceHandle], transaction);
    digitalWrite(CSs[deviceHandle], HIGH);
    return;
}
void SPICreate::pollTransmit(spi_transaction_t *transaction, int deviceHandle)
{
    spi_device_polling_transmit(handle[deviceHandle], transaction);
    return;
}
SPICREATE_END