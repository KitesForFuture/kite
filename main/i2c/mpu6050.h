#ifndef I2C_DEVICES_MPU6050
#define I2C_DEVICES_MPU6050

#include "freertos/FreeRTOS.h"
#include "interchip.h"

struct position_data {
    float accel[3];
    float gyro[3];
};

void mpu6050_init(struct i2c_identifier i2c_identifier_arg, struct position_data callibration_data,
                  uint8_t x_mapping, uint8_t y_mapping, uint8_t z_mapping,
                  bool is_x_reversed, bool is_y_reversed, bool is_z_reversed);

void mpu6050_get_position(struct position_data *out);

#endif
