#pragma once

#include "SPICREATE.h"
#include "avi_esp_libs/timeout.h"

#include <array>
#include <cstddef>
#include <cstdint>

class S25FL127S {
public:
  using JedecId = std::array<uint8_t, 3>;

  struct Status {
    bool busy{false};
    bool write_enable{false};
    bool protected_area{false};
    bool program_error{false};
    bool erase_error{false};
  };

  struct Config {
    uint32_t frequency_hz{8000000};
    avi::Timeout ready_timeout{avi::Timeout::milliseconds(1000)};
  };

  static constexpr std::size_t kPageSize = 256;
  static constexpr std::size_t kEccUnitSize = 16;
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
  [[nodiscard]] esp_err_t getStatus(Status &status);
  [[nodiscard]] esp_err_t
  eraseBlock(uint32_t address,
             avi::Timeout timeout = avi::Timeout::milliseconds(2000));
  [[nodiscard]] esp_err_t
  eraseChip(avi::Timeout timeout = avi::Timeout::milliseconds(240000));
  [[nodiscard]] esp_err_t
  write(uint32_t address, const uint8_t *data, std::size_t length = kPageSize,
        avi::Timeout timeout = avi::Timeout::milliseconds(1000));
  template <std::size_t N>
  [[nodiscard]] esp_err_t
  write(uint32_t address, const std::array<uint8_t, N> &data,
        avi::Timeout timeout = avi::Timeout::milliseconds(1000)) {
    return write(address, data.data(), data.size(), timeout);
  }
  template <std::size_t N>
  [[nodiscard]] esp_err_t
  write(uint32_t address, const uint8_t (&data)[N],
        avi::Timeout timeout = avi::Timeout::milliseconds(1000)) {
    return write(address, data, N, timeout);
  }
  [[nodiscard]] esp_err_t read(uint32_t address, uint8_t *data,
                               std::size_t length = kPageSize);
  template <std::size_t N>
  [[nodiscard]] esp_err_t read(uint32_t address, std::array<uint8_t, N> &data) {
    return read(address, data.data(), data.size());
  }
  template <std::size_t N>
  [[nodiscard]] esp_err_t read(uint32_t address, uint8_t (&data)[N]) {
    return read(address, data, N);
  }
  [[nodiscard]] esp_err_t readByte(uint32_t address, uint8_t &value);
  [[nodiscard]] esp_err_t
  writeByte(uint32_t address, uint8_t value,
            avi::Timeout timeout = avi::Timeout::milliseconds(1000));
  [[nodiscard]] std::size_t blockSize() const { return block_size_; }
  [[nodiscard]] bool initialized() const;

private:
  [[nodiscard]] esp_err_t writeEnable();
  [[nodiscard]] esp_err_t waitReadyUntil(int64_t deadline_us);
  [[nodiscard]] esp_err_t rangeErased(uint32_t address, std::size_t length,
                                      bool &erased);
  [[nodiscard]] esp_err_t blankCheckUntil(int64_t deadline_us);
  [[nodiscard]] esp_err_t eraseAddressed(uint8_t command, uint32_t address,
                                         std::size_t alignment,
                                         avi::Timeout timeout);

  // 利用中はSPIバスが本オブジェクトより長く生存する必要がある。
  SPICREATE *spi_{nullptr};
  SPICREATE::Device device_{nullptr};
  std::size_t block_size_{0};
};
