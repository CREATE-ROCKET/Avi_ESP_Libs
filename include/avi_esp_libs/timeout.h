#pragma once

#include <cstdint>
#include <limits>

namespace avi {

class Timeout {
public:
  enum class Kind : uint8_t { no_wait, finite, forever };

  [[nodiscard]] static constexpr Timeout noWait() { return Timeout{}; }
  [[nodiscard]] static constexpr Timeout milliseconds(uint64_t value) {
    return value == 0 ? noWait()
                      : Timeout{Kind::finite, Unit::milliseconds, value};
  }
  [[nodiscard]] static constexpr Timeout seconds(uint64_t value) {
    return value == 0 ? noWait() : Timeout{Kind::finite, Unit::seconds, value};
  }
  [[nodiscard]] static constexpr Timeout forever() {
    return Timeout{Kind::forever, Unit::milliseconds, 0};
  }

  [[nodiscard]] constexpr Kind kind() const { return kind_; }
  [[nodiscard]] constexpr bool isNoWait() const {
    return kind_ == Kind::no_wait;
  }
  [[nodiscard]] constexpr bool isFinite() const {
    return kind_ == Kind::finite;
  }
  [[nodiscard]] constexpr bool isForever() const {
    return kind_ == Kind::forever;
  }
  [[nodiscard]] constexpr bool millisecondsValue(uint64_t &value) const {
    if (!isFinite())
      return false;
    if (unit_ == Unit::seconds) {
      if (value_ > std::numeric_limits<uint64_t>::max() / 1000U)
        return false;
      value = value_ * 1000U;
    } else {
      value = value_;
    }
    return true;
  }

private:
  enum class Unit : uint8_t { milliseconds, seconds };
  constexpr Timeout() = default;
  constexpr Timeout(Kind kind, Unit unit, uint64_t value)
      : kind_(kind), unit_(unit), value_(value) {}

  Kind kind_{Kind::no_wait};
  Unit unit_{Unit::milliseconds};
  uint64_t value_{0};
};

} // namespace avi
