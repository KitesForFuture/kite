#include "freertos/FreeRTOS.h"
#include "cat24c256.h"

static struct i2c_identifier i2c_identifier;

union Conversion {
 float f;
 uint8_t c[sizeof(float)];
};

void cat24_init(struct i2c_identifier i2c_identifier_arg){
    i2c_identifier = i2c_identifier_arg;
    init_interchip(i2c_identifier);
}

void cat24_write_float(float number, int address){
	union Conversion conversion;
	conversion.f = number;
	i2c_send_bytes(i2c_identifier, 2, address, sizeof(float), conversion.c);
}

float cat24_read_float(int address){
	union Conversion conversion;
	i2c_read_bytes(i2c_identifier, 2, address, sizeof(float), conversion.c);
	return conversion.f;
}

// ToDoLeo Provide function to read int16_t (mpu6050 and main)