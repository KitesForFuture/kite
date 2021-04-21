#include "freertos/FreeRTOS.h"
#include "cat24c256.h"

union Conversion {
    float f;
    uint8_t c[sizeof(float)];
};

Cat24c256::Cat24c256(struct i2c_identifier i2c_identifier) : i2c_identifier{i2c_identifier} {
    init_interchip(i2c_identifier);
}

void Cat24c256::write_float(float number, int address) {
    Conversion conversion {};
    conversion.f = number;
    i2c_send_bytes(i2c_identifier, 2, address, sizeof(float), conversion.c);
}

float Cat24c256::read_float(int address) {
    Conversion conversion {};
    i2c_read_bytes(i2c_identifier, 2, address, sizeof(float), conversion.c);
    return conversion.f;
}

// ToDoLeo Provide function to read int16_t (mpu6050 and main)