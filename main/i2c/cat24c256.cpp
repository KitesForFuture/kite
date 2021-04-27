#include "freertos/FreeRTOS.h"
#include "cat24c256.h"

union Conversion {
    float f;
    uint8_t c[sizeof(float)];
};

Cat24c256::Cat24c256(struct i2c_config i2c_config) : I2cDevice(i2c_config) {}

void Cat24c256::write_float(float number, int address) {
    Conversion conversion {};
    conversion.f = number;
    send_bytes(2, address, sizeof(float), conversion.c);
}

float Cat24c256::read_float(int address) {
    Conversion conversion {};
    read_bytes(2, address, sizeof(float), conversion.c);
    return conversion.f;
}

// ToDo Leo Provide function to read int16_t (mpu6050 and main)
// ToDo Leo Provide function to read vectors
