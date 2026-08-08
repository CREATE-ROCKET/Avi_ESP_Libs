#pragma once

#include <climits>
#include <cstdint>
#include <limits>

#include "avi_esp_libs/compatibility.h"
#include "avi_esp_libs/timeout.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace avi::internal {

inline esp_err_t timeoutMilliseconds(Timeout timeout, uint64_t &milliseconds) {
  if (timeout.isForever())
    return ESP_ERR_NOT_SUPPORTED;
  if (timeout.isNoWait()) {
    milliseconds = 0;
    return ESP_OK;
  }
  return timeout.millisecondsValue(milliseconds) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

inline esp_err_t timeoutToTicks(Timeout timeout, TickType_t &ticks) {
  if (timeout.isForever()) {
    ticks = portMAX_DELAY;
    return ESP_OK;
  }
  uint64_t milliseconds{};
  const esp_err_t result = timeoutMilliseconds(timeout, milliseconds);
  if (result != ESP_OK)
    return result;
  const uint64_t whole = milliseconds / 1000U;
  const uint64_t remainder = milliseconds % 1000U;
  if (whole > (portMAX_DELAY - 1U) / configTICK_RATE_HZ)
    return ESP_ERR_INVALID_ARG;
  const uint64_t value = whole * configTICK_RATE_HZ +
                         (remainder * configTICK_RATE_HZ + 999U) / 1000U;
  if (value >= portMAX_DELAY)
    return ESP_ERR_INVALID_ARG;
  ticks = static_cast<TickType_t>(value);
  return ESP_OK;
}

inline esp_err_t timeoutToIntMilliseconds(Timeout timeout, int &milliseconds) {
  if (timeout.isForever()) {
    milliseconds = -1;
    return ESP_OK;
  }
  uint64_t value{};
  const esp_err_t result = timeoutMilliseconds(timeout, value);
  if (result != ESP_OK)
    return result;
  if (value > INT_MAX)
    return ESP_ERR_INVALID_ARG;
  milliseconds = static_cast<int>(value);
  return ESP_OK;
}

struct Deadline {
  int64_t microseconds{};
  bool forever{};
};

inline esp_err_t makeDeadline(Timeout timeout, Deadline &deadline) {
  if (timeout.isForever()) {
    deadline = {0, true};
    return ESP_OK;
  }
  uint64_t milliseconds{};
  const esp_err_t result = timeoutMilliseconds(timeout, milliseconds);
  if (result != ESP_OK ||
      milliseconds > static_cast<uint64_t>(INT64_MAX / 1000))
    return ESP_ERR_INVALID_ARG;
  const int64_t now = avi_micros();
  const int64_t delta = static_cast<int64_t>(milliseconds * 1000U);
  if (now > INT64_MAX - delta)
    return ESP_ERR_INVALID_ARG;
  deadline = {now + delta, false};
  return ESP_OK;
}

inline bool expired(const Deadline &deadline) {
  return !deadline.forever && avi_micros() >= deadline.microseconds;
}

} // namespace avi::internal
