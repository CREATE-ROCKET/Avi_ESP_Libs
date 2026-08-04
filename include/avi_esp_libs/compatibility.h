#pragma once
#include <cstdint>
#include "esp_idf_version.h"
[[nodiscard]] int64_t avi_micros();
void avi_delay_ms(uint32_t milliseconds);
