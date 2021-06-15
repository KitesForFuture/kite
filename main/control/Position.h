#ifndef ROTATION_MATRIX_FILE
#define ROTATION_MATRIX_FILE

#include "freertos/FreeRTOS.h"
#include "../i2c/Mpu6050.h"
#include "../structures/Matrix3.h"
#include "../structures/Vector3.h"

struct PositionUpdate {
    array<float, 9> rotation_matrix;
    array<float, 3> g_correction_axis;
    float g_correction_angle;
};

class Position {

    /* rotation of the drone in world coordinates
     * Columns:
     *      X - longitudinal axis (Längsachse)
     *      Y - lateral axis      (Querachse)
     *      Z - yaw axis          (Hochachse)
     * Rows:
     *      X - Real Gravitation
     *      Y/Z resemble the earths surface
     */
    array<float, 9>& matrix;
    array<float, 3> init_gravity;
    float accel_gravity_weight;

    void apply_movements(array<float, 3> gyro, float elapsed_sec, PositionUpdate& out);
    void rotate_towards_g(array<float, 3> accel, PositionUpdate& out);

public:

    Position(array<float, 9>& matrix, array<float, 3> init_gravity, float accel_gravity_weight);
    PositionUpdate update(Motion& motion, float elapsed_sec);
};

#endif
