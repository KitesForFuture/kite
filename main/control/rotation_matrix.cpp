#include <esp_log.h>
#include "rotation_matrix.h"
#include "../helpers/kitemath.h"
#include <cstring>

static const char* TAG = "RotationMatrix";

// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
void RotationMatrix::rotate_towards_g(DataVector3 accel) {

    DataVector3 axis {
        matrix.transpose_multiply_cp(init_gravity)
    };
    axis.cross_product_ip(init_gravity);
    axis.normalize();

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    // normalize accel vector
    accel.normalize();
    accel.substract_ip(init_gravity);
    // multiply by small number, so we move only tiny bit in right direction at every step -> averaging measured acceleration from vibration
    float angle = accel.get_norm() *
                  0.004;//When connected to USB, then 0.00004 suffices. When autonomous on battery 0.0004 (10 times larger) does just fine.
    // 0.00004 works, error 0.0004
    // 0.0004 works, error 0.002 except in battery mode
    // 0.004 works, error 0.01
    // 0.04, error 0.07
    // it appears that the gyro drifts a lot more when powered on battery instead of USB.
    // ToDoLeo constants / knowledge inside calcualtion.

    // rotation matrix
    DataMatrix3 difference {
        1,
        -axis.get(2) * sin(angle),
        axis.get(1) * sin(angle),
        axis.get(2) * sin(angle),
        1,
        -axis.get(0) * sin(angle),
        -axis.get(1) * sin(angle),
        axis.get(0) * sin(angle),
        1
    };

    matrix.transpose_right_multiply_ip(difference);
}

void RotationMatrix::apply_movements(DataVector3 gyro, float elapsed_ms) {

    /*
     * Convert gyro to angles (in rad).
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

    matrix.multiply_ip(difference);
}

void RotationMatrix::update(struct motion_data motion) {

    if (!timer.has_laptime()) { // ToDo improve this. It's about skipping the first time
        timer.take();
        return;
    }
    timer.take();

    apply_movements(motion.gyro, timer.get_laptime() * 0.001);
    timer.reset();

    rotate_towards_g(motion.accel);

    matrix.normalize();

    /*
    ESP_LOGI(TAG, "Values: %f,%f,%f,%f,%f,%f,%f,%f,%f", matrix[0], matrix[1],
             matrix[2], matrix[3], matrix[4], matrix[5], matrix[6],
             matrix[7], matrix[8] );
    */
}


RotationMatrix::RotationMatrix(Matrix3 matrix, DataVector3 init_gravity) : matrix{matrix}, init_gravity{init_gravity} {
    this->init_gravity.normalize();
}
