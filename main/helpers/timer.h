
#ifndef HELPERS_TIMER
#define HELPERS_TIMER

#include "freertos/FreeRTOS.h"

void delay_ms(uint32_t milliseconds);

class CycleTimer {

    int64_t start_ms;
    int64_t cycle_ms {-1};

public:

    CycleTimer();
    void end_cycle();
    float get_milliseconds();
    float get_seconds();
};

#endif