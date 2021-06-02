
#ifndef HELPERS_TIMER
#define HELPERS_TIMER

#include "freertos/FreeRTOS.h"

void delay_ms(uint32_t milliseconds);

class MsTimer {

    int64_t start_ms;
    int64_t laptime_ms {-1};

public:

    MsTimer();
    float take();
    bool has_laptime();
    float get_laptime();
    void reset();
};

#endif