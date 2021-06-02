#include "timer.h"
#include "freertos/task.h"

void delay_ms(uint32_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

MsTimer::MsTimer() {
    reset();
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

void MsTimer::reset() {
    start_ms = 0.001 * (float) esp_timer_get_time();
}
