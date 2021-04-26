#include "rotation_matrix.h"
#include "../helpers/kitemath.h"



// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
void RotationMatrix::rotate_towards_g(float mat[], float a, float b, float c) {
    // mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor'
    float tmp_vec[3];
    mat_transp_mult_vec(mat, x_gravity_factor, y_gravity_factor, z_gravity_factor, tmp_vec);

    // determine the normalized rotation axis mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' x (a,b,c)'
    float axis_1 = tmp_vec[1] * c - tmp_vec[2] * b;
    float axis_2 = tmp_vec[2] * a - tmp_vec[0] * c;
    float axis_3 = tmp_vec[0] * b - tmp_vec[1] * a;
    float norm = sqrt(axis_1 * axis_1 + axis_2 * axis_2 + axis_3 * axis_3);
    axis_1 /= norm;
    axis_2 /= norm;
    axis_3 /= norm;

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    float differenceNorm = sqrt(
            (mat[2] - a) * (mat[2] - a) + (mat[5] - b) * (mat[5] - b) + (mat[8] - c) * (mat[8] - c));
    // multiply by small number, so we move only tiny bit in right direction at every step -> averaging measured acceleration from vibration
    float angle = differenceNorm *
                  0.001;//When connected to USB, then 0.00004 suffices. When autonomous on battery 0.0004 (10 times larger) does just fine.
    // 0.00004 works, error 0.0004
    // 0.0004 works, error 0.002 except in battery mode
    // 0.004 works, error 0.01
    // 0.04, error 0.07
    // it appears that the gyro drifts a lot more when powered on battery instead of USB.
    // ToDoLeo constants / knowledge inside calcualtion.

    // rotation matrix
    float tmp_rot_matrix[9];
    tmp_rot_matrix[0] = 1;
    tmp_rot_matrix[1] = -axis_3 * sin(angle);
    tmp_rot_matrix[2] = axis_2 * sin(angle);
    tmp_rot_matrix[3] = axis_3 * sin(angle);
    tmp_rot_matrix[4] = 1;
    tmp_rot_matrix[5] = -axis_1 * sin(angle);
    tmp_rot_matrix[6] = -axis_2 * sin(angle);
    tmp_rot_matrix[7] = axis_1 * sin(angle);
    tmp_rot_matrix[8] = 1;

    mat_mult_mat_transp(mat, tmp_rot_matrix, matrix);
}

void RotationMatrix::update(struct motion_data position) {

    if (!timer.has_laptime()) { // ToDo improve this. It's about skipping the first time
        timer.take();
        return;
    }
    timer.take();

    // matrix based:
    // rotation-matrix:
    // angles in radians
    // 0.01745329 = pi/180
    float alpha = 0.01745329 * position.gyro[0] * timer.get_laptime();
    float beta = 0.01745329 * position.gyro[1] * timer.get_laptime();
    float gamma = 0.01745329 * position.gyro[2] * timer.get_laptime();

    // infinitesimal rotation matrix:
    float diff[9];
    diff[0] = 1; //maybe can replace by 1 here
    diff[1] = -sin(gamma);
    diff[2] = sin(beta);

    diff[3] = sin(gamma);
    diff[4] = 1;
    diff[5] = -sin(alpha);

    diff[6] = -sin(beta);
    diff[7] = sin(alpha);
    diff[8] = 1;

    float temp_rotation_matrix[9];
    mat_mult(matrix, diff, temp_rotation_matrix);

    rotate_towards_g(temp_rotation_matrix, position.accel[0], position.accel[1], position.accel[2]);

    normalize_matrix(matrix);
}


RotationMatrix::RotationMatrix(float gravity[]) {
    normalize(gravity, 3);
    x_gravity_factor = gravity[0];
    y_gravity_factor = gravity[1];
    z_gravity_factor = gravity[2];
}

void RotationMatrix::print() {
    printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", matrix[0], matrix[1],
           matrix[2], matrix[3], matrix[4], matrix[5], matrix[6],
           matrix[7], matrix[8]);
}
