#pragma once
#include "SPICREATE.h"
#include <array>
#include <cstdint>

class ICM42688 {
public:
  using Data = std::array<int16_t, 6>;
  ICM42688() = default;
  ~ICM42688();
  ICM42688(const ICM42688 &) = delete;
  ICM42688 &operator=(const ICM42688 &) = delete;
  ICM42688(ICM42688 &&) = delete;
  ICM42688 &operator=(ICM42688 &&) = delete;
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t get(Data &data);

private:
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
};
