#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/mpu6050.h"
#include "../helpers/timer.h"
#include "../structures/Matrix3.h"
#include "../structures/Vector3.h"
#include "../structures/DataMatrix3.h"
#include "../structures/DataVector3.h"


class RotationMatrix {

    float x_gravity_factor, y_gravity_factor, z_gravity_factor;
    MsTimer timer {};
    // rotation of the drone in world coordinates
    Matrix3 matrix;

    void apply_movements(Vector3 gyro, float elapsed_ms);
    void rotate_towards_g(Vector3 accel);

public:

    explicit RotationMatrix(float* matrix, float gravity[]);
    void update(struct motion_data position);
};

#endif
