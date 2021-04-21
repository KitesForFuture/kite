#ifndef I2C_DEVICES_CAT24C256
#define I2C_DEVICES_CAT24C256

#include "interchip.h"

class Cat24c256 {

    struct i2c_identifier i2c_identifier;

public:

    Cat24c256(struct i2c_identifier i2c_identifier_arg);
    void write_float(float number, int address);
    float read_float(int address);

};

#endif