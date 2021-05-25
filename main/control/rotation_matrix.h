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

    MsTimer timer {};
    // rotation of the drone in world coordinates
    Matrix3 matrix;
    DataVector3 init_gravity;

    void apply_movements(DataVector3 gyro, float elapsed_ms);
    void rotate_towards_g(DataVector3 accel);

public:

    explicit RotationMatrix(Matrix3 matrix, DataVector3 init_gravity);
    void update(motion_data motion);
};

#endif
