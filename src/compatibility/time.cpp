#include "avi_esp_libs/compatibility.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int64_t avi_micros() { return esp_timer_get_time(); }

void avi_delay_ms(uint32_t milliseconds) {
  if (milliseconds == 0) {
    taskYIELD();
    return;
  }

  const int64_t deadline_us =
      esp_timer_get_time() + static_cast<int64_t>(milliseconds) * 1000;

  for (;;) {
    const int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0)
      return;

    const uint64_t remaining_ms =
        (static_cast<uint64_t>(remaining_us) + 999U) / 1000U;
    const TickType_t ticks = pdMS_TO_TICKS(remaining_ms);
    if (ticks > 1) {
      // tick境界で早く復帰してもdeadlineを再確認できるよう、最後の1 tickは残す。
      vTaskDelay(ticks - 1);
      continue;
    }

    // データシートの最小待機時間を満たすため、最後の短時間だけbusy waitする。
    esp_rom_delay_us(static_cast<uint32_t>(remaining_us));
  }
}
