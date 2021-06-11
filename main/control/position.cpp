#include "position.h"
#include "cmath"

// Align model gravitation with measured gravitation.
// Measured gravitation results from averaging accel values over many iterations.
void Position::rotate_towards_g(array<float, 3> accel) {

    // Compute rotation axis
    array<float, 3> axis = Matrix3::transpose_multiply(matrix, init_gravity);
    axis = Vector3::cross_product(axis, init_gravity);
    Vector3::normalize(axis);

    // Compute rotation angle
    Vector3::normalize(accel);
    accel = Vector3::subtract(accel, init_gravity);
    float angle = Vector3::get_norm(accel) * accel_gravity_weight;

    // Compute infinitesimal rotation matrix from given axis and angle
    array<float, 9> difference {
        1,
        -axis[2] * angle,
        axis[1] * angle,
        axis[2] * angle,
        1,
        -axis[0] * angle,
        -axis[1] * angle,
        axis[0] * angle,
        1
    };

    // Apply
    matrix = Matrix3::transpose_right_multiply(matrix, difference);
}

// Calculation new position based on gyro measurements
void Position::apply_movements(array<float, 3> gyro, float elapsed_sec) {

    // Convert gyro to angles (in rad)
    gyro = Vector3::multiply(gyro, (M_PI/180) * elapsed_sec);

    // Infinitesimal rotation matrix
    array<float, 9> difference {
        1,
        -gyro[2],
        gyro[1],
        gyro[2],
        1,
        -gyro[0],
        -gyro[1],
        gyro[0],
        1
    };

    // Apply
    matrix = Matrix3::multiply(matrix, difference);
}

void Position::update(Motion& motion, float elapsed_sec) {
    apply_movements(motion.gyro, elapsed_sec);
    rotate_towards_g(motion.accel);
    Matrix3::normalize(matrix);
}


Position::Position(array<float, 9>& matrix, array<float, 3> init_gravity, float accel_gravity_weight)
: matrix{matrix}, init_gravity{init_gravity}, accel_gravity_weight{accel_gravity_weight} {
    // The Gravity vector is the direction the gravitational force is supposed to point in KITE COORDINATES with the nose pointing to the sky
    Vector3::normalize(this->init_gravity);
}
