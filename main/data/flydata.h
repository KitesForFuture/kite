//
// Created by Leonard Koll on 16.05.21.
//

#ifndef KITE_FLYDATA_H
#define KITE_FLYDATA_H

#include <array>
#include "../i2c/mpu6050.h"
#include "../control/position.h"

using namespace std;

struct Flydata {

    float cycle_sec;

    // Sensor values
    float height;
    Motion motion;

    // Calculated state
    PositionUpdate update;
    array<float, 9> position;

};


#endif //KITE_FLYDATA_H
