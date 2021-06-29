//
// Created by Leonard Koll on 14.06.21.
//

/*
 * Next:
 * - getHoverHeightControl
 * - fly
 * - ToDos
 */

#include "HoverController.h"
#include "math.h"

HoverController::HoverController(array<float, 3> normalized_gravitation) : FlightController(normalized_gravitation) {}

void HoverController::fly(array<float, 9>& position_matrix, array<float, 3>& gyro) {

    float elevon_angle {
        get_angle(
                position_matrix, gyro,
                1, 2,
                backwards_tilt_angle,
                elevon_p_factor, elevon_d_factor
            )
    };
    // ToDo: Verify if semantic change is ok. PID used to came AFTER this:
    if (fabs(elevon_p_factor * (last_elevon_angle - elevon_angle)) < 1) {
        elevon_angle = last_elevon_angle;
    } else {
        last_elevon_angle = elevon_angle;
    }

    float rudder_angle {
            get_angle(
                    position_matrix, gyro,
                    2, 1,
                    sidewards_tilt_angle,
                    rudder_p_factor, rudder_d_factor
            )
    };
    // ToDo: Verify if semantic change is ok. PID used to came AFTER this:
    if (fabs(rudder_p_factor * (last_rudder_angle - rudder_angle)) < 1) {
        rudder_angle = last_rudder_angle;
    } else {
        last_rudder_angle = rudder_angle;
    }

}

float HoverController::get_angle(array<float, 9>& position_matrix, array<float, 3>& gyro, int relevant_axis_index, int other_axis_index, float tilt_angle, float p_factor, float d_factor) {

    array<float, 3> relevant_axis { Matrix3::get(position_matrix, relevant_axis_index, true) };
    array<float, 3> other_axis { Matrix3::get(position_matrix, other_axis_index, true) };

    float delta {0};
    // IF MORE KITE-LIKE THAN ROLL NEUTRAL, DON'T CONTROLL ELEVATOR, BUT ONLY RUDDER (and vice versa)
    if (Matrix3::get(position_matrix,0,0) > 0.1
        || fabs(relevant_axis[0]) < fabs(other_axis[0]))
    {
        delta = get_position_delta(position_matrix, relevant_axis);
        delta -= tilt_angle;
    }
    // ToDo Vorzeichen ist bei unterschiedlichen aufrufen unterscheidlich
    return p_factor * delta + d_factor * gyro[relevant_axis_index];
}

float HoverController::get_position_delta(array<float, 9>& position_matrix, array<float,3>& relevant_axis) {

    // neutral_position is where the left wing (the back) would land if only rudder (elevon) is used.
    // ToDo In case of Elevator (position matrix col index 1), normalized graviation was negative.
    array<float, 3> neutral_position {
        Vector3::cross_product(relevant_axis, normalized_gravitation)
    };
    Vector3::normalize(neutral_position);

    //  1:	going left      (flying belly up)
    //  0:	going straight  (going straight up)
    // -1:	going right     (flying belly down)
    array<float, 3> roll_axis = Matrix3::get(position_matrix, 0, true);
    float orientation {
        Vector3::scalar_product(roll_axis, neutral_position)
    };

    /*  Calculate current angle
     *  Note:
     *      position_matrix(0,0)== 1  => nose straight up
     *      position_matrix(0,0)== 0  => nose horizontal
     *      position_matrix(0,0)==-1  => nose straight down
     */
    if (Matrix3::get(position_matrix,0,0) > 0) {
        return safe_acos(orientation) - 0.5*M_PI;
    }
    if (orientation > 0) { // going left (up)
        return -0.5*M_PI - safe_acos(orientation);
    } else {
        return 1.5*M_PI - safe_acos(orientation);
    }
}
