//
// Created by Leonard Koll on 14.06.21.
//

#ifndef KITE_LANDINGCONTROLLER_H
#define KITE_LANDINGCONTROLLER_H


#include "FlightController.h"


class LandingController: public FlightController {

public:

    explicit LandingController(array<float, 3> normalized_gravitation);
    ControlParameters get_control_parameters(array<float, 9>& position_matrix, array<float, 3>& gyro, float height, float height_derivative, float elapsed_sec) override;
    bool is_done() override;

};



#endif //KITE_LANDINGCONTROLLER_H
