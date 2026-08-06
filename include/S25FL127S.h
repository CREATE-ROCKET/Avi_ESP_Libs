#pragma once
#include "SPICREATE.h"
#include <cstddef>
#include <cstdint>
class S25FL127S {
public:
  static constexpr size_t kPageSize = 256;
  S25FL127S() = default;
  ~S25FL127S();
  S25FL127S(const S25FL127S &) = delete;
  S25FL127S &operator=(const S25FL127S &) = delete;
  S25FL127S(S25FL127S &&) = delete;
  S25FL127S &operator=(S25FL127S &&) = delete;
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t erase(uint32_t timeout_ms = 120000);
  [[nodiscard]] esp_err_t write(uint32_t address, const uint8_t *data,
                                size_t length = kPageSize,
                                uint32_t timeout_ms = 1000);
  [[nodiscard]] esp_err_t read(uint32_t address, uint8_t *data,
                               size_t length = kPageSize);

private:
  [[nodiscard]] esp_err_t waitReady(uint32_t timeout_ms);
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
};
