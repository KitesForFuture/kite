#ifndef I2C_DEVICES_BMP280
#define I2C_DEVICES_BMP280

#include "interchip.h"

int bmp280_update_if_possible();

void bmp280_init(struct i2c_identifier i2c_identifier_arg, float minus_dp_by_dt);

float bmp280_get_pressure_diff();

float bmp280_get_height();

#endif
