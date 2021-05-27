#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/mpu6050.h"
#include "../helpers/timer.h"
#include "../structures/Matrix3.h"
#include "../structures/Vector3.h"


class RotationMatrix {

    MsTimer timer {};
    // rotation of the drone in world coordinates
    array<float, 9>* matrix_data_ptr;
    Matrix3 matrix;
    array<float, 3> init_gravity_data;
    Vector3 init_gravity;

    void apply_movements(array<float, 3> gyro, float elapsed_ms);
    void rotate_towards_g(array<float, 3> accel);

public:

    explicit RotationMatrix(array<float, 9>* matrix, array<float, 3> init_gravity);
    void update(Motion motion);
};

#endif
