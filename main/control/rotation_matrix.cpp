#include "rotation_matrix.h"
#include "../helpers/kitemath.h"

// rotates matrix mat such that mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' aligns more with (a,b,c)'
// (x_gravity_factor, y_gravity_factor, z_gravity_factor) can be initially measured acceleration vector, usually something close to (0,0,1)
// (a,b,c) can be the currently measured acceleration vector
void RotationMatrix::rotate_towards_g(Vector3 kite_gravitation) {

    // determine the normalized rotation axis mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' x (a,b,c)'
    Vector3 axis = matrix.multiply(world_up, true).cross(kite_gravitation);
    axis.normalize();

    // determine the approximate angle between mat'*(x_gravity_factor, y_gravity_factor, z_gravity_factor)' and (a,b,c)'
    Vector3 difference {
        matrix[0][2] - kite_gravitation[0],
        matrix[1][2] - kite_gravitation[1],
        matrix[2][2] - kite_gravitation[2]
    };

    // multiply by small number, so we move only tiny bit in right direction at every step -> averaging measured acceleration from vibration
    float angle = difference.get_norm() *
                  0.001;//When connected to USB, then 0.00004 suffices. When autonomous on battery 0.0004 (10 times larger) does just fine.
    // 0.00004 works, error 0.0004
    // 0.0004 works, error 0.002 except in battery mode
    // 0.004 works, error 0.01
    // 0.04, error 0.07
    // it appears that the gyro drifts a lot more when powered on battery instead of USB.
    // ToDoLeo constants / knowledge inside calcualtion.

    // rotation matrix
    Matrix3 tmp_rot_matrix {
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

    matrix = matrix.multiply(tmp_rot_matrix, false, true);
}

void RotationMatrix::update(MotionData position) {

    if (!timer.has_laptime()) { // ToDo improve this. It's about skipping the first time
        timer.take();
        return;
    }
    timer.take();

    // 0.01745329 = pi/180
    Vector3 radian_angles { position.gyro.multiply(0.01745329 * timer.get_laptime()) };

    // infinitesimal rotation matrix:
    Matrix3 diff {
        1,
        -sin(radian_angles[2]),
        sin(radian_angles[1]),
        sin(radian_angles[2]),
        1,
        -sin(radian_angles[0]),
        -sin(radian_angles[1]),
        sin(radian_angles[0]),
        1
    };

    matrix = matrix.multiply(diff);
    rotate_towards_g(position.accel);
    matrix.normalize();
}


RotationMatrix::RotationMatrix(Vector3 up) : world_up{up} {
    world_up.normalize();
}

void RotationMatrix::print() {
    printf("rotation matrix:\n %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n",
           matrix[0][0], matrix[0][1], matrix[0][2],
           matrix[1][1], matrix[1][1], matrix[1][2],
           matrix[2][0], matrix[2][1], matrix[2][2]);
}
