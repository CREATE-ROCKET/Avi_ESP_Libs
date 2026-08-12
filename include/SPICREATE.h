#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "avi_esp_libs/timeout.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class AS5047D;
class H3LIS331;
class ICM20602;
class ICM20948;
class ICM42688;
class LPS25HB;
class S25FL127S;
class S25FL512S;

class SPICREATE {
public:
  struct Config {
    spi_host_device_t host{SPI2_HOST};
    int sck{12};
    int miso{13};
    int mosi{11};
    std::size_t max_transfer_size{SPI_MAX_DMA_LEN};
    avi::Timeout transaction_timeout{avi::Timeout::noWait()};
  };

  SPICREATE() = default;
  ~SPICREATE();
  SPICREATE(const SPICREATE &) = delete;
  SPICREATE &operator=(const SPICREATE &) = delete;
  SPICREATE(SPICREATE &&) = delete;
  SPICREATE &operator=(SPICREATE &&) = delete;

  [[nodiscard]] esp_err_t begin(const Config &config);
  [[nodiscard]] esp_err_t
  begin(spi_host_device_t host = SPI2_HOST, int sck = 12, int miso = 13,
        int mosi = 11, std::size_t max_transfer_size = SPI_MAX_DMA_LEN);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] std::size_t deviceCount() const;

private:
  class LockGuard;
  using Device = spi_device_handle_t;
  struct DeviceConfig {
    int chip_select{-1};
    uint32_t frequency_hz{0};
    uint8_t mode{0};
    uint8_t queue_size{1};
    uint8_t cs_ena_posttrans{0};
    uint8_t cs_ena_pretrans{0};
  };

  friend class AS5047D;
  friend class H3LIS331;
  friend class ICM20602;
  friend class ICM20948;
  friend class ICM42688;
  friend class LPS25HB;
  friend class S25FL127S;
  friend class S25FL512S;

  [[nodiscard]] esp_err_t addDevice(const DeviceConfig &config, Device &device);
  [[nodiscard]] esp_err_t removeDevice(Device &device);
  [[nodiscard]] esp_err_t transmit(Device device,
                                   spi_transaction_t &transaction);
  [[nodiscard]] esp_err_t pollingTransmit(Device device,
                                          spi_transaction_t &transaction);
  [[nodiscard]] esp_err_t readRegister(Device device, uint8_t address,
                                       uint8_t &value);
  [[nodiscard]] esp_err_t writeRegister(Device device, uint8_t address,
                                        uint8_t value);
  [[nodiscard]] esp_err_t read(Device device, uint8_t command, uint8_t *data,
                               std::size_t length);
  [[nodiscard]] esp_err_t sendCommand(Device device, uint8_t command);
  [[nodiscard]] esp_err_t takeBusLock();
  void giveBusLock();
  [[nodiscard]] bool owns(Device device) const;
  [[nodiscard]] std::size_t maxTransferSize() const {
    return max_transfer_size_;
  }

  static constexpr std::size_t kMaxDevices = 8;
  spi_host_device_t host_{SPI2_HOST};
  std::size_t max_transfer_size_{0};
  avi::Timeout transaction_timeout_{avi::Timeout::noWait()};
  SemaphoreHandle_t bus_lock_{nullptr};
  bool initialized_{false};
  std::array<Device, kMaxDevices> devices_{};
};
