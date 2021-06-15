//
// Created by Leonard Koll on 14.06.21.
//

#ifndef KITE_LANDINGCONTROLLER_H
#define KITE_LANDINGCONTROLLER_H


#include "FlightController.h"

class LandingController: public FlightController {
public:

    void fly() override;

};


#endif //KITE_LANDINGCONTROLLER_H
