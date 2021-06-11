#include "timer.h"
#include "freertos/task.h"

void delay_ms(uint32_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

CycleTimer::CycleTimer() {
    start = esp_timer_get_time();
}

void CycleTimer::end_cycle() {
    int64_t now = esp_timer_get_time();
    cycle_sec = 0.000001 * (now - start);
    start = now;
}

float CycleTimer::get_seconds() {
    return cycle_sec;
}
