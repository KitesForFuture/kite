#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/mpu6050.h"

void rotation_matrix_update(struct position_data position, float rotation_matrix[]);

void rotation_matrix_init (int8_t x_gravity, int8_t y_gravity, int8_t z_gravity);

#endif
