//
// Created by Leonard Koll on 14.06.21.
//

#ifndef KITE_MANUALCONTROLLER_H
#define KITE_MANUALCONTROLLER_H


#include "FlightController.h"

class ManualController: public FlightController {
public:

    void fly() override;

};


#endif //KITE_MANUALCONTROLLER_H
