#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/mpu6050.h"
#include "../structures/Matrix3.h"
#include "../structures/Vector3.h"


class Position {

    // rotation of the drone in world coordinates
    array<float, 9>& matrix;
    array<float, 3> init_gravity;
    float accel_gravity_weight;

    void apply_movements(array<float, 3> gyro, float elapsed_sec);
    void rotate_towards_g(array<float, 3> accel);

public:

    Position(array<float, 9>& matrix, array<float, 3> init_gravity, float accel_gravity_weight);
    void update(Motion& motion, float elapsed_sec);
};

#endif