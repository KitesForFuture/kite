#include "freertos/FreeRTOS.h"
#include "cat24c256.h"

static struct i2c_identifier i2c_identifier;

union Conversion {
 float f;
 uint8_t c[sizeof(float)];
};

void init_cat24(struct i2c_identifier i2c_identifier_arg){ // ToDoLeo rename to init. Make sure init calls in main don't conflict
	i2c_identifier = i2c_identifier_arg;
  init_interchip(i2c_identifier);
}

// ToDoLeo rename
void write2EEPROM(float number, int address){
	union Conversion conversion;
	conversion.f = number;
	i2c_send_bytes(i2c_identifier, 2, address, sizeof(float), conversion.c);
}

// ToDoLeo rename
float readEEPROM(int address){	
	union Conversion conversion;
	i2c_read_bytes(i2c_identifier, 2, address, sizeof(float), conversion.c);
	return conversion.f;
}

// ToDoLeo Provide function to read int16_t (mpu6050 and main)