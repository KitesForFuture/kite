
#ifndef HELPERS_TIMER
#define HELPERS_TIMER

#include "freertos/FreeRTOS.h"

void delay_ms(uint32_t milliseconds);

class CycleTimer {

    int64_t start;
    float cycle_sec {-1};

public:

    CycleTimer();
    void end_cycle();
    void measure();
    float get_seconds();
};

#endif