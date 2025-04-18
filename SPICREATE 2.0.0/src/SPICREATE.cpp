// version: 2.0.0
#include "SPICREATE.h" // 2.0.0
/** @deprecated csセット用だったが、spi_device_interface_config_tのspics_numで代用することにした */
void csSet(spi_transaction_t *t)
{
    return;
}
/** @deprecated csセット用だったが、spi_device_interface_config_tのspics_numで代用することにした */
void csReset(spi_transaction_t *t)
{
    return;
}
SPICREATE_BEGIN

#if !(IS_S3)
/**
 *  @brief SPICreateを利用するときに最初に実行する関数
 *  ESP32 無印のみ対応の関数としている
 */
bool SPICreate::begin(uint8_t spi_bus, int8_t sck, int8_t miso, int8_t mosi, uint32_t unused)
{
    spi_host_device_t host_in;
    host_in = (spi_bus == HSPI) ? HSPI_HOST : VSPI_HOST;
    return begin(host_in, sck, miso, mosi);
}
#endif

bool SPICreate::begin(spi_host_device_t host_in, int8_t sck, int8_t miso, int8_t mosi)
{
    if ((sck == -1) && (miso == -1) && (mosi == -1))
    {
#if (IS_S3)
        sck = 12;
        miso = 13;
        mosi = 11;
#else
        sck = (host_in == VSPI_HOST) ? VSPI_IOMUX_PIN_NUM_CLK : HSPI_IOMUX_PIN_NUM_CLK;
        miso = (host_in == VSPI_HOST) ? VSPI_IOMUX_PIN_NUM_MISO : HSPI_IOMUX_PIN_NUM_MISO;
        mosi = (host_in == VSPI_HOST) ? VSPI_IOMUX_PIN_NUM_MOSI : HSPI_IOMUX_PIN_NUM_MOSI;
#endif
    }
    bus_cfg.sclk_io_num = sck;
    bus_cfg.miso_io_num = miso;
    bus_cfg.mosi_io_num = mosi;
    bus_cfg.quadwp_io_num = -1; // unused
    bus_cfg.quadhd_io_num = -1; // unused
    bus_cfg.max_transfer_sz = max_size;

    esp_err_t e = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
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
    if (deviceNum > 3)
    {
        return 0;
    }
    if_cfg->spics_io_num = cs;
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
    pollTransmit(&comm, deviceHandle);
}
uint8_t SPICreate::readByte(uint8_t addr, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    comm.tx_data[0] = addr;
    comm.length = 16;
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
    spi_device_transmit(handle[deviceHandle], transaction);
    return;
}
void SPICreate::pollTransmit(spi_transaction_t *transaction, int deviceHandle)
{
    spi_device_polling_transmit(handle[deviceHandle], transaction);
    return;
}
SPICREATE_END