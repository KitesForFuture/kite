//
// Created by Leonard Koll on 14.06.21.
//

#ifndef KITE_HOVERCONTROLLER_H
#define KITE_HOVERCONTROLLER_H


#include "FlightController.h"
#include "../structures/Matrix3.h"
#include "../structures/Vector3.h"

struct HoverControllerConfig {
    float backwards_tilt_angle;
    float sidewards_tilt_angle;
    float elevon_p_factor;
    float elevon_d_factor;
    float rudder_p_factor;
    float rudder_d_factor;
    float propeller_p_factor;
    float propeller_d_factor;
    float target_height_bound_meters;
    float neutral_propeller_speed;
    float goal_height_meters;
    float rate_of_climb;
};

class HoverController: public FlightController {

    HoverControllerConfig config;
    float last_elevon_angle {0};
    float last_rudder_angle {0};
    float target_height {0};

    float get_position_delta(array<float, 9>& position_matrix, array<float,3>& relevant_axis);
    float get_angle(array<float, 9>& position_matrix, array<float, 3>& gyro, int relevant_axis_index, int other_axis_index, float tilt_angle, float p_factor, float d_factor);
    float get_propeller_speed(float h, float d_h, float elapsed_sec);

public:

    explicit HoverController(array<float, 3> normalized_gravitation, HoverControllerConfig config);
    ControlParameters get_control_parameters(array<float, 9>& position_matrix, array<float, 3>& gyro, float height, float height_derivative, float elapsed_sec) override;

};


#endif //KITE_HOVERCONTROLLER_H
