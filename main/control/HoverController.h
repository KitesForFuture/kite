//
// Created by Leonard Koll on 14.06.21.
//

#ifndef KITE_HOVERCONTROLLER_H
#define KITE_HOVERCONTROLLER_H


#include "FlightController.h"

class HoverController: public FlightController {
public:

    void fly() override;

};


#endif //KITE_HOVERCONTROLLER_H
