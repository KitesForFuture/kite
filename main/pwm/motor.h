#ifndef PWM_MOTOR
#define PWM_MOTOR
#include <cmath>

#define FREQ_HZ 200
#define DUTY_RESOLUTION 13

class Motor {

    static double duty_interval_micro_seconds;

    static int next_free_channel;

    int channel;
    int min_pulse_width_micro_seconds;
    int max_pulse_width_micro_seconds;

public:

    Motor(int gpio, int min_pulse_width_micro_seconds, int max_pulse_width_micro_seconds);
    void set(double spectrum_ratio); // [0-1] to select [min_pulse_width_microseconds - max_pulse_width_micro_seconds]

};

#endif
