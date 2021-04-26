#include "freertos/FreeRTOS.h"
#include "motor.h"
#include "driver/ledc.h"

int Motor::next_free_channel {0};
double Motor::duty_interval_micro_seconds { ((float)1000000/(float)FREQ_HZ) / pow(2, DUTY_RESOLUTION) };

Motor::Motor(int gpio, int min_pulse_width_micro_seconds, int max_pulse_width_micro_seconds) :
min_pulse_width_micro_seconds {min_pulse_width_micro_seconds}, max_pulse_width_micro_seconds {max_pulse_width_micro_seconds}
{

    // All motors/servos work with the same frequency and duty resolution
    // Hence it's fine to use the same timer for all (we use LEDC_TIMER_0)
    // Reapplying configuration on each initialisation is redundant but ok for the given use-case
    ledc_timer_config_t timer_config {
            .speed_mode = LEDC_HIGH_SPEED_MODE,      // timer mode
            .duty_resolution = static_cast<ledc_timer_bit_t>(DUTY_RESOLUTION),     // resolution of PWM duty in bit
            .timer_num = LEDC_TIMER_0,              // timer index
            .freq_hz = FREQ_HZ,                     // frequency of PWM signal
            .clk_cfg = LEDC_AUTO_CLK                // Auto select the source clock
    };
    ledc_timer_config(&timer_config);

    // Get the next free channel and wire it with GPIO and timer.
    channel = next_free_channel++;
    ledc_channel_config_t channel_config {
            .gpio_num   = gpio,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel    = static_cast<ledc_channel_t>(channel),
            .intr_type  = {},
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
    };
    ledc_channel_config(&channel_config);
}

void Motor::set(double spectrum_ratio) {
    auto pulse_width_micro_seconds = (max_pulse_width_micro_seconds - min_pulse_width_micro_seconds) * spectrum_ratio + min_pulse_width_micro_seconds;
    auto duty = pulse_width_micro_seconds / duty_interval_micro_seconds;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(channel), static_cast<uint32_t>(duty));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(channel));
}
