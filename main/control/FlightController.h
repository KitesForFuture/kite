//
// Created by Leonard Koll on 14.06.21.
//

#ifndef KITE_FLIGHTCONTROLLER_H
#define KITE_FLIGHTCONTROLLER_H

#include <array>
#include "Position.h"

using namespace std;

struct ControlParameters {
    float angle_elevon;
    float angle_rudder;
    float speed_propeller;
};

class FlightController {

protected:

    array<float, 3> normalized_gravitation;

    float safe_acos(float x);

public:

    explicit FlightController(array<float, 3> normalized_gravitation);
    virtual ControlParameters get_control_parameters(array<float, 9>& position_matrix, array<float, 3>& gyro, float height, float height_derivative, float elapsed_sec) = 0;
    virtual bool is_done();

};


#endif //KITE_FLIGHTCONTROLLER_H
