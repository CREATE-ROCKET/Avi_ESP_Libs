#pragma once
#include <array>
#include <cstdint>
#include "SPICREATE.h"
class H3LIS331 {
public: using Data=std::array<int16_t,3>; ~H3LIS331(); [[nodiscard]] esp_err_t begin(SPICREATE& spi,int chip_select,uint32_t frequency=8000000); [[nodiscard]] esp_err_t end(); [[nodiscard]] esp_err_t whoAmI(uint8_t& value); [[nodiscard]] esp_err_t get(Data& data);
private: SPICREATE* spi_{nullptr}; SPICREATE::Device device_{nullptr};
};
