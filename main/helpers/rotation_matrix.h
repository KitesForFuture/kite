#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include <string.h>
#include "math.h"
#include "../i2c_devices/mpu6050.h"
#include "timer.h"

float rotation_matrix[9];

void updateRotationMatrix();


#endif
