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
};

class HoverController: public FlightController {

    HoverControllerConfig config;
    float last_elevon_angle {0};
    float last_rudder_angle {0};

    float get_position_delta(array<float, 9>& position_matrix, array<float,3>& relevant_axis, bool switch_sign);
    float get_angle(array<float, 9>& position_matrix, array<float, 3>& gyro, int relevant_axis_index, int other_axis_index, float tilt_angle, float p_factor, float d_factor, bool switch_sign);

public:

    explicit HoverController(array<float, 3> normalized_gravitation, HoverControllerConfig config);
    void fly(array<float, 9>& position_matrix, array<float, 3>& gyro) override;

};


#endif //KITE_HOVERCONTROLLER_H
