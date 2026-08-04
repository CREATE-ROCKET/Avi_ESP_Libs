#include "SPICREATE.h"

#include <algorithm>

#include "driver/gpio.h"

SPICREATE::~SPICREATE() { (void)end(); }

esp_err_t SPICREATE::begin(spi_host_device_t host, int sck, int miso, int mosi,
                           std::size_t max_transfer_size) {
    if (initialized_) return ESP_ERR_INVALID_STATE;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(sck) || !GPIO_IS_VALID_OUTPUT_GPIO(mosi) ||
        !GPIO_IS_VALID_GPIO(miso) || max_transfer_size == 0) return ESP_ERR_INVALID_ARG;
    spi_bus_config_t config{};
    config.sclk_io_num = sck;
    config.miso_io_num = miso;
    config.mosi_io_num = mosi;
    config.quadwp_io_num = -1;
    config.quadhd_io_num = -1;
    config.max_transfer_sz = static_cast<int>(max_transfer_size);
    const esp_err_t result = spi_bus_initialize(host, &config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK) return result;
    host_ = host;
    initialized_ = true;
    devices_.fill(nullptr);
    return ESP_OK;
}

esp_err_t SPICREATE::end() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    for (Device& device : devices_) {
        if (device != nullptr) {
            const esp_err_t result = spi_bus_remove_device(device);
            if (result != ESP_OK) return result;
            device = nullptr;
        }
    }
    const esp_err_t result = spi_bus_free(host_);
    if (result == ESP_OK) initialized_ = false;
    return result;
}

esp_err_t SPICREATE::addDevice(const spi_device_interface_config_t& config, int chip_select,
                               Device& device) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(chip_select)) return ESP_ERR_INVALID_ARG;
    auto slot = std::find(devices_.begin(), devices_.end(), nullptr);
    if (slot == devices_.end()) return ESP_ERR_NO_MEM;
    spi_device_interface_config_t local = config;
    local.spics_io_num = chip_select;
    const esp_err_t result = spi_bus_add_device(host_, &local, &device);
    if (result == ESP_OK) *slot = device;
    return result;
}

esp_err_t SPICREATE::removeDevice(Device device) {
    if (!initialized_ || device == nullptr) return ESP_ERR_INVALID_STATE;
    auto slot = std::find(devices_.begin(), devices_.end(), device);
    if (slot == devices_.end()) return ESP_ERR_NOT_FOUND;
    const esp_err_t result = spi_bus_remove_device(device);
    if (result == ESP_OK) *slot = nullptr;
    return result;
}

esp_err_t SPICREATE::transmit(Device device, spi_transaction_t& transaction) {
    if (!initialized_ || device == nullptr) return ESP_ERR_INVALID_STATE;
    return spi_device_transmit(device, &transaction);
}

esp_err_t SPICREATE::pollingTransmit(Device device, spi_transaction_t& transaction) {
    if (!initialized_ || device == nullptr) return ESP_ERR_INVALID_STATE;
    return spi_device_polling_transmit(device, &transaction);
}

esp_err_t SPICREATE::readRegister(Device device, uint8_t address, uint8_t& value) {
    spi_transaction_t transaction{};
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 16;
    transaction.tx_data[0] = address;
    const esp_err_t result = pollingTransmit(device, transaction);
    if (result == ESP_OK) value = transaction.rx_data[1];
    return result;
}

esp_err_t SPICREATE::writeRegister(Device device, uint8_t address, uint8_t value) {
    spi_transaction_t transaction{};
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 16;
    transaction.tx_data[0] = address;
    transaction.tx_data[1] = value;
    return pollingTransmit(device, transaction);
}

esp_err_t SPICREATE::sendCommand(Device device, uint8_t command) {
    spi_transaction_t transaction{};
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8;
    transaction.tx_data[0] = command;
    return pollingTransmit(device, transaction);
}
