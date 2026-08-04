#include "avi_esp_libs/compatibility.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
int64_t avi_micros() { return esp_timer_get_time(); }
void avi_delay_ms(uint32_t milliseconds) {
  if (milliseconds == 0) {
    taskYIELD();
    return;
  }
  TickType_t ticks = pdMS_TO_TICKS(milliseconds);
  if (ticks == 0)
    ticks = 1;
  vTaskDelay(ticks);
}
