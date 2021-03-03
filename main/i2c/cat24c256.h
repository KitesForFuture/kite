#ifndef I2C_DEVICES_CAT24C256
#define I2C_DEVICES_CAT24C256

#include "interchip.h"

void cat24_init(struct i2c_identifier i2c_identifier_arg);

void cat24_write_float(float number, int address);

float cat24_read_float(int address);

#endif