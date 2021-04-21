#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/mpu6050.h"
#include "../helpers/timer.h"

class RotationMatrix {

    int8_t x_gravity_factor, y_gravity_factor, z_gravity_factor;
    MsTimer timer {};
    // rotation of the drone in world coordinates
    float matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    void rotate_towards_g(float mat[], float a, float b, float c);

public:

    explicit RotationMatrix(float gravity[]);
    void update(struct motion_data position);
    void print(); // ToDo Leo remove: Logging instead.
};

#endif
