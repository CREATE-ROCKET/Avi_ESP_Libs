#pragma once

#include <cstddef>
#include <cstdint>

#include "SPICREATE.h"

inline esp_err_t avi_spi_add(SPICREATE &spi, int cs, uint32_t frequency,
                             uint8_t mode, SPICREATE::Device &device) {
  spi_device_interface_config_t config{};
  config.clock_speed_hz = static_cast<int>(frequency);
  config.mode = mode;
  config.queue_size = 1;
  return spi.addDevice(config, cs, device);
}

inline esp_err_t avi_spi_read(SPICREATE &spi, SPICREATE::Device device,
                              uint8_t command, uint8_t *data,
                              std::size_t length) {
  if (data == nullptr || length == 0)
    return ESP_ERR_INVALID_ARG;
  spi_transaction_ext_t transaction{};
  transaction.base.flags = SPI_TRANS_VARIABLE_CMD;
  transaction.base.cmd = command;
  transaction.base.length = length * 8;
  transaction.base.rx_buffer = data;
  transaction.command_bits = 8;
  return spi.pollingTransmit(device, transaction.base);
}
