#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include <string.h>
#include "../helpers/math.h"
#include "../i2c_devices/mpu6050.h"
#include "../helpers/timer.h"

float* rotation_matrix;
float gyro_in_kite_coords[3];

void updateRotationMatrix();
void initRotationMatrix(float* matrix);


#endif
