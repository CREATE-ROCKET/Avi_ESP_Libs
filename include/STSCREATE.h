#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "avi_esp_libs/timeout.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class STS3215;

class STSCREATE {
public:
  enum class Instruction : uint8_t {
    ping = 0x01,
    read = 0x02,
    write = 0x03,
    reg_write = 0x04,
    action = 0x05,
    recovery = 0x06,
    reset = 0x0A,
    sync_read = 0x82,
    sync_write = 0x83
  };
  enum class DirectionPolarity : uint8_t { tx_high, tx_low };
  enum class Baudrate : uint32_t {
    bps1000000 = 1000000,
    bps500000 = 500000,
    bps250000 = 250000,
    bps128000 = 128000,
    bps115200 = 115200,
    bps76800 = 76800,
    bps57600 = 57600,
    bps38400 = 38400
  };
  struct Config {
    uart_port_t port{UART_NUM_1};
    int tx{GPIO_NUM_NC};
    int rx{GPIO_NUM_NC};
    int direction_enable{GPIO_NUM_NC};
    DirectionPolarity direction_polarity{DirectionPolarity::tx_high};
    Baudrate baudrate{Baudrate::bps1000000};
    avi::Timeout lock_timeout{avi::Timeout::noWait()};
    avi::Timeout response_timeout{avi::Timeout::milliseconds(10)};
  };

  STSCREATE() = default;
  ~STSCREATE();
  STSCREATE(const STSCREATE &) = delete;
  STSCREATE &operator=(const STSCREATE &) = delete;
  STSCREATE(STSCREATE &&) = delete;
  STSCREATE &operator=(STSCREATE &&) = delete;

  [[nodiscard]] esp_err_t begin(const Config &config);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] esp_err_t ping(uint8_t id, uint8_t *device_error = nullptr);
  [[nodiscard]] esp_err_t read(uint8_t id, uint8_t address, uint8_t *data,
                               std::size_t length,
                               uint8_t *device_error = nullptr);
  [[nodiscard]] esp_err_t write(uint8_t id, uint8_t address,
                                const uint8_t *data, std::size_t length,
                                bool wait_response = true,
                                uint8_t *device_error = nullptr);
  [[nodiscard]] esp_err_t regWrite(uint8_t id, uint8_t address,
                                   const uint8_t *data, std::size_t length,
                                   bool wait_response = true,
                                   uint8_t *device_error = nullptr);
  [[nodiscard]] esp_err_t action(uint8_t id = 0xFE, bool wait_response = false,
                                 uint8_t *device_error = nullptr);
  [[nodiscard]] esp_err_t syncRead(uint8_t address, uint8_t length,
                                   const uint8_t *ids, std::size_t id_count,
                                   uint8_t *data, std::size_t data_size);
  [[nodiscard]] esp_err_t syncWrite(uint8_t address, uint8_t length,
                                    const uint8_t *ids, std::size_t id_count,
                                    const uint8_t *data, std::size_t data_size);
  [[nodiscard]] esp_err_t recovery(uint8_t id, uint8_t *device_error = nullptr);
  [[nodiscard]] esp_err_t resetState(uint8_t id,
                                     uint8_t *device_error = nullptr);

private:
  class LockGuard;
  friend class STS3215;
  [[nodiscard]] esp_err_t
  transaction(uint8_t id, Instruction instruction, const uint8_t *parameters,
              std::size_t parameter_count, bool wait_response,
              uint8_t *response_data, std::size_t response_length,
              uint8_t *device_error);
  [[nodiscard]] esp_err_t sendPacket(uint8_t id, Instruction instruction,
                                     const uint8_t *parameters,
                                     std::size_t parameter_count);
  [[nodiscard]] esp_err_t receivePacket(uint8_t expected_id, uint8_t *data,
                                        std::size_t expected_length,
                                        uint8_t *device_error);
  [[nodiscard]] esp_err_t takeBusLock();
  void giveBusLock();
  [[nodiscard]] esp_err_t setTransmitDirection(bool transmit);
  [[nodiscard]] static bool validId(uint8_t id, bool allow_broadcast);

  uart_port_t port_{UART_NUM_1};
  int direction_enable_{GPIO_NUM_NC};
  DirectionPolarity direction_polarity_{DirectionPolarity::tx_high};
  avi::Timeout lock_timeout_{avi::Timeout::noWait()};
  avi::Timeout response_timeout_{avi::Timeout::milliseconds(10)};
  SemaphoreHandle_t bus_lock_{nullptr};
  bool initialized_{false};
};
