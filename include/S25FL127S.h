#pragma once

#include "SPICREATE.h"

#include <array>
#include <cstddef>
#include <cstdint>

class S25FL127S {
public:
  using JedecId = std::array<uint8_t, 3>;

  struct Config {
    uint32_t frequency_hz{8000000};
    uint32_t ready_timeout_ms{1000};
  };

  static constexpr std::size_t kPageSize = 256;
  static constexpr uint32_t kCapacity = 16U * 1024U * 1024U;
  static constexpr JedecId kExpectedJedecId{0x01, 0x20, 0x18};

  S25FL127S() = default;
  ~S25FL127S();
  S25FL127S(const S25FL127S &) = delete;
  S25FL127S &operator=(const S25FL127S &) = delete;
  S25FL127S(S25FL127S &&) = delete;
  S25FL127S &operator=(S25FL127S &&) = delete;

  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                const Config &config);
  [[nodiscard]] esp_err_t begin(SPICREATE &spi, int chip_select,
                                uint32_t frequency_hz = 8000000);
  [[nodiscard]] esp_err_t end();
  [[nodiscard]] esp_err_t readJedecId(JedecId &id);
  [[nodiscard]] esp_err_t readStatus(uint8_t &status);
  [[nodiscard]] esp_err_t erase(uint32_t timeout_ms = 240000);
  [[nodiscard]] esp_err_t write(uint32_t address, const uint8_t *data,
                                std::size_t length = kPageSize,
                                uint32_t timeout_ms = 1000);
  [[nodiscard]] esp_err_t read(uint32_t address, uint8_t *data,
                               std::size_t length = kPageSize);
  [[nodiscard]] bool initialized() const;

private:
  [[nodiscard]] esp_err_t writeEnable();
  [[nodiscard]] esp_err_t waitReadyUntil(int64_t deadline_us);

  // 利用中はSPIバスが本オブジェクトより長く生存する必要がある。
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  uint32_t ready_timeout_ms_{1000};
};
