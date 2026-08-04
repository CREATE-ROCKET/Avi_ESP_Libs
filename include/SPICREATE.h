#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

class SPICREATE {
public:
  using Device = spi_device_handle_t;

  SPICREATE() = default;
  ~SPICREATE();
  SPICREATE(const SPICREATE &) = delete;
  SPICREATE &operator=(const SPICREATE &) = delete;

  [[nodiscard]] esp_err_t
  begin(spi_host_device_t host = SPI2_HOST, int sck = 12, int miso = 13,
        int mosi = 11, std::size_t max_transfer_size = SPI_MAX_DMA_LEN);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t addDevice(const spi_device_interface_config_t &config,
                                    int chip_select, Device &device);
  [[nodiscard]] esp_err_t removeDevice(Device device);
  [[nodiscard]] esp_err_t transmit(Device device,
                                   spi_transaction_t &transaction);
  [[nodiscard]] esp_err_t pollingTransmit(Device device,
                                          spi_transaction_t &transaction);
  [[nodiscard]] esp_err_t readRegister(Device device, uint8_t address,
                                       uint8_t &value);
  [[nodiscard]] esp_err_t writeRegister(Device device, uint8_t address,
                                        uint8_t value);
  [[nodiscard]] esp_err_t sendCommand(Device device, uint8_t command);
  [[nodiscard]] bool initialized() const { return initialized_; }

private:
  static constexpr std::size_t kMaxDevices = 8;
  spi_host_device_t host_{SPI2_HOST};
  bool initialized_{false};
  std::array<Device, kMaxDevices> devices_{};
};
