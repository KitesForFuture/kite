#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../helpers/timer.h"
#include "bmp280.h"

#define  UPDATE_INTERVAL_MS 50
#define  SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT 0.2
#define  SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT 0.2
#define  INITIAL_MEASUREMENT_CYCLE_COUNT 5
#define  WARM_UP_PERIOD_MS 500

static struct i2c_identifier i2c_identifier;
static float minus_dp_by_dt_factor;
static float initial_smoothed_temperature = 0;
static float initial_smoothed_pressure = 0;
static float current_smoothed_temperature = 0;
static float current_smoothed_pressure = 0;
static Time last_update = 0;

static void start_measurement() {
    // chip_addr, register, precision(0x25 least precise, takes 9 ms, 0x5D most precise, takes 45ms)
    i2c_send_byte(i2c_identifier, 1, 0xF4, 0x5D);
    last_update = start_timer();
}

static uint32_t get_temperature() {
    uint8_t result[3];
    i2c_read_bytes(i2c_identifier, 1, 0xFA, 3, result);
    return (uint32_t) ((result[0] << 16) | (result[1] << 8) | result[2]);
}

static float get_pressure() {
    uint8_t result[3];
    i2c_read_bytes(i2c_identifier, 1, 0xF7, 3, result);
    uint32_t bmp280_raw_pressure_reading = (uint32_t) ((result[0] << 16) | (result[1] << 8) | result[2]);
    return 1365.3 - 0.00007555555555 * (float) (bmp280_raw_pressure_reading);
}

int bmp280_update_if_possible() {
    if (query_timer_ms(last_update) >= UPDATE_INTERVAL_MS) {
        // current_smoothed_temperature = 0.2 * (float)getTemperature() + 0.8 * current_smoothed_temperature;
        current_smoothed_temperature = SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT * (float) get_temperature() +
                                         (1 - SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT) * current_smoothed_temperature;
        current_smoothed_pressure = SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT * get_pressure() +
                                      (1 - SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT) * current_smoothed_pressure;

        start_measurement();
        return 1;
    }
    return 0;
}

void bmp280_init(struct i2c_identifier i2c_identifier_arg, float minus_dp_by_dt) {
    i2c_identifier = i2c_identifier_arg;
    init_interchip(i2c_identifier);
    minus_dp_by_dt_factor = minus_dp_by_dt;
    delay_ms(WARM_UP_PERIOD_MS);

    // Setup current values to be not 0 (that would worsen the following smoothening process)
    start_measurement();
    delay_ms(UPDATE_INTERVAL_MS);
    current_smoothed_temperature = (float) get_temperature();
    current_smoothed_pressure = get_pressure();

    // Setup smoothed values
    start_measurement();
    for (int i = 0; i < INITIAL_MEASUREMENT_CYCLE_COUNT; i++) {
        delay_ms(UPDATE_INTERVAL_MS);
        bmp280_update_if_possible();
    }
    initial_smoothed_temperature = current_smoothed_temperature;
    initial_smoothed_pressure = current_smoothed_pressure;
}

// DIFFERENCE IN ATMOSPHERIC PRESSURE SINCE BOOT
float bmp280_get_pressure_diff() {
    float delta_temperature = current_smoothed_temperature - initial_smoothed_temperature;
    float delta_pressure = current_smoothed_pressure - initial_smoothed_pressure;
    return delta_pressure + delta_temperature * minus_dp_by_dt_factor;
}

// HEIGHT DIFFERENCE SINCE BOOT AS MEASURED USING ATMOSPHERIC PRESSURE
float bmp280_get_height() {
    return -10 * bmp280_get_pressure_diff();
}
