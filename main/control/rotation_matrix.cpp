#include <esp_log.h>
#include "rotation_matrix.h"
#include "../helpers/kitemath.h"
#include <cstring>

static const char* TAG = "RotationMatrix";

// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
void RotationMatrix::rotate_towards_g(Matrix3 mat, Vector3 accel) {
    // mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor'
    float tmp_vec[3];
    mat_transp_mult_vec(mat, x_gravity_factor, y_gravity_factor, z_gravity_factor, tmp_vec);

    DataVector3 axis {
        mat
    };

    // determine the normalized rotation axis mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' x (a,b,c)'
    float axis_1 = tmp_vec[1] * c - tmp_vec[2] * b;
    float axis_2 = tmp_vec[2] * a - tmp_vec[0] * c;
    float axis_3 = tmp_vec[0] * b - tmp_vec[1] * a;
    float norm = sqrt(axis_1 * axis_1 + axis_2 * axis_2 + axis_3 * axis_3);
    axis_1 /= norm;
    axis_2 /= norm;
    axis_3 /= norm;

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    // normalize accel vector
    norm = sqrt(a * a + b * b + c * c);
    a = a/norm;
    b = b/norm;
    c = c/norm;
    float differenceNorm = sqrt(
            (x_gravity_factor - a) * (x_gravity_factor - a) + (y_gravity_factor - b) * (y_gravity_factor - b) + (z_gravity_factor - c) * (z_gravity_factor - c));
    // multiply by small number, so we move only tiny bit in right direction at every step -> averaging measured acceleration from vibration
    float angle = differenceNorm *
                  0.004;//When connected to USB, then 0.00004 suffices. When autonomous on battery 0.0004 (10 times larger) does just fine.
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

void RotationMatrix::apply_movements(DataVector3 gyro, float elapsed_ms) {

    /*
     * (1) Convert gyro to angles (in rad).
     * Note that gyro is of type DataVector3 (not Vector3) and hence a copy.
     * 0.01745329 = pi/180
     */
    gyro.multiply_ip(0.01745329 * elapsed_ms);

    // infinitesimal rotation matrix:
    DataMatrix3 difference {
        1,
        -sin(gyro.get(2)),
        sin(gyro.get(1)),
        sin(gyro.get(2)),
        1,
        -sin(gyro.get(0)),
        -sin(gyro.get(1)),
        sin(gyro.get(0)),
        1
    };

    matrix.multiply_ip(difference); // ToDo das muss implementiert und das unten entfernt werden.

    float temp_rotation_matrix[9];
    mat_mult(matrix, diff, temp_rotation_matrix);
}

void RotationMatrix::update(struct motion_data motion) {

    if (!timer.has_laptime()) { // ToDo improve this. It's about skipping the first time
        timer.take();
        return;
    }
    timer.take();

    apply_movements(motion.gyro, timer.get_laptime() * 0.001)
    timer.reset();




    rotate_towards_g(temp_rotation_matrix, position.accel[0], position.accel[1], position.accel[2]);

    normalize_matrix(matrix);

    /*
    ESP_LOGI(TAG, "Values: %f,%f,%f,%f,%f,%f,%f,%f,%f", matrix[0], matrix[1],
             matrix[2], matrix[3], matrix[4], matrix[5], matrix[6],
             matrix[7], matrix[8] );
    */
}


RotationMatrix::RotationMatrix(float matrix[], float gravity[]) : matrix{matrix} {
    normalize(gravity, 3);
    x_gravity_factor = gravity[0];
    y_gravity_factor = gravity[1];
    z_gravity_factor = gravity[2];
}
