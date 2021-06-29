//
// Created by Leonard Koll on 14.06.21.
//

/*
 * Next:
 * - getHoverHeightControl
 * - fly
 */

#include "HoverController.h"
#include "math.h"

HoverController::HoverController(array<float, 3> normalized_gravitation, HoverControllerConfig config) :
        FlightController(normalized_gravitation), config{config}
{}

ControlParameters HoverController::get_control_parameters(array<float, 9>& position_matrix, array<float, 3>& gyro) {

    float angle_elevon {
        get_angle(
                position_matrix, gyro,
                1, 2,
                config.backwards_tilt_angle,
                config.elevon_p_factor, config.elevon_d_factor
            )
    };
    if (fabs(config.elevon_p_factor * (last_elevon_angle - angle_elevon)) < 1) {
        angle_elevon = last_elevon_angle;
    } else {
        last_elevon_angle = angle_elevon;
    }
    printf("Elevon angle %f\n", angle_elevon);

    float angle_rudder {
            get_angle(
                    position_matrix, gyro,
                    2, 1,
                    config.sidewards_tilt_angle,
                    config.rudder_p_factor, config.rudder_d_factor
            )
    };
    if (fabs(last_rudder_angle - angle_rudder) < 1) {
        angle_rudder = last_rudder_angle;
    } else {
        last_rudder_angle = angle_rudder;
    }

    return ControlParameters {
        angle_elevon,
        angle_rudder,
        0,
        0,
    };
}

// ToDo improve so that switch_sign can be removed
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
    return p_factor * delta + d_factor * gyro[relevant_axis_index];
}

float HoverController::get_position_delta(array<float, 9>& position_matrix, array<float,3>& relevant_axis) {

    // neutral_position is where the left wing (the back) would land if only rudder (elevon) is used.
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
