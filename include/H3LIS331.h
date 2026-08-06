#pragma once
#include "SPICREATE.h"
#include <array>
#include <cstdint>
class H3LIS331 {
public:
  using Data = std::array<int16_t, 3>;
  H3LIS331() = default;
  ~H3LIS331();
  H3LIS331(const H3LIS331 &) = delete;
  H3LIS331 &operator=(const H3LIS331 &) = delete;
  H3LIS331(H3LIS331 &&) = delete;
  H3LIS331 &operator=(H3LIS331 &&) = delete;
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t get(Data &data);

private:
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
};
