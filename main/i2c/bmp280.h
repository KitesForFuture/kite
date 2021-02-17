#ifndef I2C_DEVICES_BMP280
#define I2C_DEVICES_BMP280

#include "interchip.h"

int update_bmp280_if_necessary();

void init_bmp280(struct i2c_identifier i2c_identifier_arg, float minus_dp_by_dt);

float getPressureDiff();

float getHeight();

#endif
