#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmp280.h"

#define  UPDATE_INTERVAL_MS 50
#define  SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT 0.2
#define  SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT 0.2
#define  INITIAL_MEASUREMENT_CYCLE_COUNT 5
#define  WARM_UP_PERIOD_MS 500

void Bmp280::start_measurement() {
    // chip_addr, register, precision(0x25 least precise, takes 9 ms, 0x5D most precise, takes 45ms)
    send_byte(1, 0xF4, 0x5D);
}

uint32_t Bmp280::get_temperature() {
    uint8_t result[3];
    read_bytes(1, 0xFA, 3, result);
    return (uint32_t) ((result[0] << 16) | (result[1] << 8) | result[2]);
}

float Bmp280::get_pressure() {
    uint8_t result[3];
    read_bytes(1, 0xF7, 3, result);
    uint32_t bmp280_raw_pressure_reading = (uint32_t) ((result[0] << 16) | (result[1] << 8) | result[2]);
    return 1365.3 - 0.00007555555555 * (float) (bmp280_raw_pressure_reading);
}

Bmp280::Bmp280(struct i2c_config i2c_config, float minus_dp_by_dt) : I2cDevice(i2c_config), minus_dp_by_dt {minus_dp_by_dt} {
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
        update_if_possible();
    }
    initial_smoothed_temperature = current_smoothed_temperature;
    initial_smoothed_pressure = current_smoothed_pressure;
}

int Bmp280::update_if_possible() {
    if (timer.take() >= UPDATE_INTERVAL_MS) {
        // current_smoothed_temperature = 0.2 * (float)getTemperature() + 0.8 * current_smoothed_temperature;
        current_smoothed_temperature = SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT * (float) get_temperature() +
                                       (1 - SMOOTHING_TEMPERATURE_RECENT_VALUE_WEIGHT) * current_smoothed_temperature;
        current_smoothed_pressure = SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT * get_pressure() +
                                    (1 - SMOOTHING_PRESSURE_RECENT_VALUE_WEIGHT) * current_smoothed_pressure;

        timer = {};
        start_measurement();
        return 1;
    }
    return 0;
}

// DIFFERENCE IN ATMOSPHERIC PRESSURE SINCE BOOT
float Bmp280::get_pressure_diff() {
    float delta_temperature = current_smoothed_temperature - initial_smoothed_temperature;
    float delta_pressure = current_smoothed_pressure - initial_smoothed_pressure;
    return delta_pressure + delta_temperature * minus_dp_by_dt;
}

// HEIGHT DIFFERENCE SINCE BOOT AS MEASURED USING ATMOSPHERIC PRESSURE
float Bmp280::get_height() {
    return -10 * get_pressure_diff();
}
