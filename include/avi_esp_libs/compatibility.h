#pragma once
#include <cstdint>
#include "driver/uart.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
constexpr uart_sclk_t AVI_UART_DEFAULT_CLOCK = UART_SCLK_DEFAULT;
#else
constexpr uart_sclk_t AVI_UART_DEFAULT_CLOCK = UART_SCLK_APB;
#endif
[[nodiscard]] int64_t avi_micros();
void avi_delay_ms(uint32_t milliseconds);
