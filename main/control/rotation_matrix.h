#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/mpu6050.h"
#include "../helpers/timer.h"
#include "../helpers/Vector3.h"
#include "../helpers/Matrix3.h"

class RotationMatrix {

    Vector3 gravitation_factor;
    MsTimer timer {};
    Matrix3 matrix {1, 0, 0, 0, 1, 0, 0, 0, 1};// rotation of the drone in world coordinates

    void rotate_towards_g(Vector3 gravitation);

public:

    explicit RotationMatrix(Vector3 gravitation);
    void update(MotionData position);
    void print(); // ToDo Leo remove: Logging instead.
};

#endif
