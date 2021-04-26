#ifndef I2C_DEVICES_CAT24C256
#define I2C_DEVICES_CAT24C256

#include "i2c_device.h"

class Cat24c256: protected I2cDevice {
public:

    explicit Cat24c256(struct i2c_config i2c_config);
    void write_float(float number, int address);
    float read_float(int address);

};

#endif