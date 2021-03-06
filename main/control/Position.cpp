#include "Position.h"
#include "cmath"

// Align model gravitation with measured gravitation.
// Measured gravitation results from averaging accel values over many iterations.
void Position::rotate_towards_g(array<float, 3> accel, PositionUpdate& out) {

    // acceleration was measured by a sensor mounted on the kite, hence:
    array<float, 3> accel_in_world_coordinates {Matrix3::multiply(matrix, accel)};

    // Compute rotation axis
    out.g_correction_axis = Vector3::cross_product(accel_in_world_coordinates, init_gravity);
    Vector3::normalize(out.g_correction_axis, 0.000001);

    // Compute rotation angle
    Vector3::normalize(accel_in_world_coordinates);
    array<float, 3> rotation_vector { Vector3::subtract(accel_in_world_coordinates, init_gravity) };
    out.g_correction_angle = Vector3::get_norm(rotation_vector) * accel_gravity_weight;

    // Compute infinitesimal rotation matrix from given axis and angle
    array<float, 9> difference {
        1,
        -out.g_correction_axis[2] * out.g_correction_angle,
        out.g_correction_axis[1] * out.g_correction_angle,
        out.g_correction_axis[2] * out.g_correction_angle,
        1,
        -out.g_correction_axis[0] * out.g_correction_angle,
        -out.g_correction_axis[1] * out.g_correction_angle,
        out.g_correction_axis[0] * out.g_correction_angle,
        1
    };

    // Apply
    matrix = Matrix3::multiply(difference, matrix);
}

// Calculation new position based on gyro measurements
void Position::apply_movements(array<float, 3> gyro, float elapsed_sec, PositionUpdate& out) {

    // Convert gyro to angles (in rad)
    gyro = Vector3::multiply(gyro, (M_PI/180) * elapsed_sec);

    // Infinitesimal rotation matrix
    out.rotation_matrix = {
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
    matrix = Matrix3::multiply(matrix, out.rotation_matrix);
}

PositionUpdate Position::update(Motion& motion, float elapsed_sec) {

    PositionUpdate update {};

    apply_movements(motion.gyro, elapsed_sec, update);
    rotate_towards_g(motion.accel, update);
    Matrix3::normalize(matrix);

    return update;
}


Position::Position(array<float, 9>& matrix, array<float, 3> init_gravity, float accel_gravity_weight)
: matrix{matrix}, init_gravity{init_gravity}, accel_gravity_weight{accel_gravity_weight} {
    // The Gravity vector is the direction the gravitational force is supposed to point in KITE COORDINATES with the nose pointing to the sky
    Vector3::normalize(this->init_gravity);
}
