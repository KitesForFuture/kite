#include "timer.h"
#include "freertos/task.h"

void delay_ms(uint32_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

CycleTimer::CycleTimer() {
    start_ms = 0.001 * (float) esp_timer_get_time();
}

void CycleTimer::end_cycle() {
    cycle_ms = 0.001 * (float)esp_timer_get_time() - start_ms;
    start_ms = 0.001 * (float) esp_timer_get_time();
}

float CycleTimer::get_milliseconds() {
    return cycle_ms;
}

float CycleTimer::get_seconds() {
    return cycle_ms * 0.001;
}
