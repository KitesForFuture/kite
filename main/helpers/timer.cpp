#include "timer.h"
#include "freertos/task.h"

void delay_ms(int64_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

MsTimer::MsTimer() {
    start_ms = 0.001 * (float) esp_timer_get_time();
}

float MsTimer::take() {
    laptime_ms = 0.001 * (float)esp_timer_get_time() - start_ms;
    return laptime_ms;
}

bool MsTimer::has_laptime() {
    return laptime_ms > 0;
}

float MsTimer::get_laptime() {
    return laptime_ms;
}
