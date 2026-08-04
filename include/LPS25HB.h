#pragma once
#include "SPICREATE.h"
#include <cstdint>
class LPS25HB {
public:
  struct Data {
    uint32_t raw{0};
    int32_t pascals{0};
  };
  ~LPS25HB();
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t whoAmI(uint8_t &value);
  [[nodiscard]] esp_err_t get(Data &data);

private:
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
};
