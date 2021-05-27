#include <esp_log.h>
#include "rotation_matrix.h"
#include "../helpers/kitemath.h"
#include <cstring>

static const char* TAG = "RotationMatrix";

// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
void RotationMatrix::rotate_towards_g(array<float, 3> accel_data) {

    array<float, 3> axis_data = matrix.transpose_multiply(init_gravity);
    Vector3 axis {&axis_data};
    axis_data = axis.cross_product(init_gravity);
    axis.normalize();

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    // normalize accel vector
    Vector3 accel {&accel_data};
    accel.normalize();
    accel_data = accel.subtract(init_gravity);
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
    array<float, 9> difference {
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

    *matrix_data_ptr = matrix.transpose_right_multiply(Matrix3{&difference});
}

void RotationMatrix::apply_movements(array<float, 3> gyro_data, float elapsed_ms) {

    Vector3 gyro {&gyro_data};
    /*
     * Convert gyro to angles (in rad).
     * Note that gyro is of type DataVector3 (not Vector3) and hence a copy.
     * 0.01745329 = pi/180
     */
    gyro_data = gyro.multiply(0.01745329 * elapsed_ms);

    // infinitesimal rotation matrix:
    array<float, 9> difference {
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

    *matrix_data_ptr = matrix.multiply(Matrix3{&difference});
}

void RotationMatrix::update(Motion motion) {

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


RotationMatrix::RotationMatrix(array<float, 9>* matrix_data_ptr, array<float, 3> init_gravity_data) : matrix_data_ptr{matrix_data_ptr}, matrix{Matrix3{matrix_data_ptr}}, init_gravity_data{init_gravity_data}, init_gravity{Vector3{&this->init_gravity_data}} {
    Vector3{&this->init_gravity_data}.normalize();
}
