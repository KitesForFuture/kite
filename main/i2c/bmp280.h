#ifndef I2C_DEVICES_BMP280
#define I2C_DEVICES_BMP280

#include "interchip.h"
#include "../helpers/timer.h"

class Bmp280 {

    struct i2c_identifier i2c_identifier;
    float minus_dp_by_dt;
    float initial_smoothed_temperature = 0;
    float initial_smoothed_pressure = 0;
    float current_smoothed_temperature = 0;
    float current_smoothed_pressure = 0;
    Time last_update = 0;

    void start_measurement();
    uint32_t get_temperature();
    float get_pressure();

public:

    Bmp280(struct i2c_identifier i2c_identifier_arg, float minus_dp_by_dt);
    int update_if_possible();
    float get_pressure_diff();
    float get_height();

};

#endif
