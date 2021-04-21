#include "timer.h"
#include "freertos/task.h"

static Time start_time_for_uptime = 0;

Time start_timer() {
    return esp_timer_get_time();
}

// time since "timer" in seconds
float query_timer_seconds(int64_t time) {
    int64_t current_time = esp_timer_get_time();
    return 0.000001 * (float) (current_time - time);
}

int64_t query_timer_ms(int64_t time) {
    return (esp_timer_get_time() - time) * 0.001;
}

void init_uptime() {
    start_time_for_uptime = start_timer();
}

float get_uptime_seconds() {
    return query_timer_seconds(start_time_for_uptime);
}

void delay_ms(int64_t milliseconds) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}