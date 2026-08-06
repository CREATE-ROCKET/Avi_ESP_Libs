#pragma once
#include "SPICREATE.h"
#include <array>
#include <cstdint>
class ICM20602 {
public:
  using Data = std::array<int16_t, 6>;
  ICM20602() = default;
  ~ICM20602();
  ICM20602(const ICM20602 &) = delete;
  ICM20602 &operator=(const ICM20602 &) = delete;
  ICM20602(ICM20602 &&) = delete;
  ICM20602 &operator=(ICM20602 &&) = delete;
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t get(Data &data);

private:
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
};
