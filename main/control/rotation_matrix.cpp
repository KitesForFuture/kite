#include <esp_log.h>
#include "rotation_matrix.h"
#include "../helpers/kitemath.h"
#include <cstring>

static const char* TAG = "RotationMatrix";

// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
void RotationMatrix::rotate_towards_g(array<float, 3> accel) {

    array<float, 3> axis = Matrix3::transpose_multiply(matrix, init_gravity);
    axis = Vector3::cross_product(axis, init_gravity);
    Vector3::normalize(axis);

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    // normalize accel vector
    Vector3::normalize(accel);
    accel = Vector3::subtract(accel, init_gravity);
    // multiply by small number, so we move only tiny bit in right direction at every step -> averaging measured acceleration from vibration
    float angle = Vector3::get_norm(accel) *
                  0.004;//When connected to USB, then 0.00004 suffices. When autonomous on battery 0.0004 (10 times larger) does just fine.
    // 0.00004 works, error 0.0004
    // 0.0004 works, error 0.002 except in battery mode
    // 0.004 works, error 0.01
    // 0.04, error 0.07
    // it appears that the gyro drifts a lot more when powered on battery instead of USB.
    // ToDoLeo constants / knowledge inside calcualtion.

    // rotation matrix
    array<float, 9> difference {
        1,
        -axis[2] * sin(angle),
        axis[1] * sin(angle),
        axis[2] * sin(angle),
        1,
        -axis[0] * sin(angle),
        -axis[1] * sin(angle),
        axis[0] * sin(angle),
        1
    };

    matrix = Matrix3::transpose_right_multiply(matrix, difference);
}

void RotationMatrix::apply_movements(array<float, 3> gyro, float elapsed_sec) {

    /*
     * Convert gyro to angles (in rad).
     * Note that gyro is of type DataVector3 (not Vector3) and hence a copy.
     * 0.01745329 = pi/180
     */
    gyro = Vector3::multiply(gyro, 0.01745329 * elapsed_sec);

    // infinitesimal rotation matrix:
    array<float, 9> difference {
        1,
        -sin(gyro[2]),
        sin(gyro[1]),
        sin(gyro[2]),
        1,
        -sin(gyro[0]),
        -sin(gyro[1]),
        sin(gyro[0]),
        1
    };

    matrix = Matrix3::multiply(matrix, difference);
}

void RotationMatrix::update(Motion& motion) {

    if (!timer.has_laptime()) { // ToDo improve this. It's about skipping the first time
        timer.take();
        return;
    }
    timer.take();

    apply_movements(motion.gyro, timer.get_laptime() * 0.001);
    timer.reset();

    rotate_towards_g(motion.accel);

    Matrix3::normalize(matrix);
}


RotationMatrix::RotationMatrix(array<float, 9>& matrix, array<float, 3> init_gravity) : matrix{matrix}, init_gravity{init_gravity} {
    Vector3::normalize(this->init_gravity);
}
