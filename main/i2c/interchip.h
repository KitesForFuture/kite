#ifndef I2C_DEVICES_INTERCHIP
#define I2C_DEVICES_INTERCHIP

#include "freertos/FreeRTOS.h"

struct i2c_bus {
  int sda;
  int scl;
};

struct i2c_identifier {
  struct i2c_bus bus;
  int chip_addr;
  int does_increment_address_on_write; // On read, address is always auto incremented
};

void init_interchip(struct i2c_identifier device);

void i2c_send_bytes(struct i2c_identifier device, int data_addr_len, uint16_t data_addr, int data_len, uint8_t data[]);

void i2c_send_byte(struct i2c_identifier device, int data_addr_len, uint16_t data_addr, uint8_t data);

void i2c_read_bytes(struct i2c_identifier device, int data_addr_len, uint16_t data_addr, int data_len, uint8_t out[]);

#endif