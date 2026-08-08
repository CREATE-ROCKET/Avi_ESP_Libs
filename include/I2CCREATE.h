#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "avi_esp_libs/timeout.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/i2c_types.h"
#else
#include "driver/i2c.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class LPS25HB;
class SSCDRRN005PD2A5;

class I2CCREATE {
public:
  struct Config {
    i2c_port_t port{I2C_NUM_0};
    int sda{8};
    int scl{9};
    uint32_t frequency_hz{100000};
    bool enable_internal_pullups{false};
    avi::Timeout lock_timeout{avi::Timeout::noWait()};
    avi::Timeout operation_timeout{avi::Timeout::milliseconds(20)};
  };

  I2CCREATE() = default;
  ~I2CCREATE();
  I2CCREATE(const I2CCREATE &) = delete;
  I2CCREATE &operator=(const I2CCREATE &) = delete;
  I2CCREATE(I2CCREATE &&) = delete;
  I2CCREATE &operator=(I2CCREATE &&) = delete;

  [[nodiscard]] esp_err_t begin(const Config &config);
  [[nodiscard]] esp_err_t begin(i2c_port_t port, int sda, int scl,
                                uint32_t frequency_hz = 100000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] uint32_t frequencyHz() const { return frequency_hz_; }
  [[nodiscard]] std::size_t deviceCount() const;
  [[nodiscard]] esp_err_t probe(uint8_t address);

private:
  class LockGuard;
  using Device = uint8_t;
  struct DeviceConfig {
    uint8_t address{0};
  };
  struct DeviceSlot {
    bool used{false};
    uint8_t address{0};
    void *handle{nullptr};
  };

  friend class LPS25HB;
  friend class SSCDRRN005PD2A5;

  [[nodiscard]] esp_err_t addDevice(const DeviceConfig &config, Device &device);
  [[nodiscard]] esp_err_t removeDevice(Device &device);
  [[nodiscard]] esp_err_t write(Device device, const uint8_t *data,
                                std::size_t length);
  [[nodiscard]] esp_err_t read(Device device, uint8_t *data,
                               std::size_t length);
  [[nodiscard]] esp_err_t writeRead(Device device, const uint8_t *write_data,
                                    std::size_t write_length,
                                    uint8_t *read_data,
                                    std::size_t read_length);
  [[nodiscard]] esp_err_t writeRegister(Device device, uint8_t address,
                                        uint8_t value);
  [[nodiscard]] esp_err_t readRegister(Device device, uint8_t address,
                                       uint8_t &value);
  [[nodiscard]] esp_err_t readRegisters(Device device, uint8_t address,
                                        uint8_t *data, std::size_t length);
  [[nodiscard]] esp_err_t takeBusLock();
  void giveBusLock();
  [[nodiscard]] DeviceSlot *slot(Device device);

  static constexpr Device kInvalidDevice = 0xFF;
  static constexpr std::size_t kMaxDevices = 8;
  i2c_port_t port_{I2C_NUM_0};
  uint32_t frequency_hz_{0};
  avi::Timeout lock_timeout_{avi::Timeout::noWait()};
  avi::Timeout operation_timeout_{avi::Timeout::milliseconds(20)};
  SemaphoreHandle_t bus_lock_{nullptr};
  void *bus_handle_{nullptr};
  bool initialized_{false};
  std::array<DeviceSlot, kMaxDevices> devices_{};
};
