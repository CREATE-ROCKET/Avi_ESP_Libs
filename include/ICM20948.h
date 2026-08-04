#pragma once
#include <array>
#include <cstdint>
#include "SPICREATE.h"
class ICM20948 {
public:
    struct Data { std::array<int16_t,6> imu{}; std::array<int16_t,3> magnetic{}; };
    ~ICM20948();
    [[nodiscard]] esp_err_t begin(SPICREATE& spi,int chip_select,uint32_t frequency=8000000);
    [[nodiscard]] esp_err_t end();
    [[nodiscard]] esp_err_t whoAmI(uint8_t& value);
    [[nodiscard]] esp_err_t get(Data& data);
private: [[nodiscard]] esp_err_t selectBank(uint8_t bank); SPICREATE* spi_{nullptr}; SPICREATE::Device device_{nullptr};
};
