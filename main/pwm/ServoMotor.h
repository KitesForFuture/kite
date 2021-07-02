//
// Created by Leonard Koll on 29.06.21.
//

#ifndef KITE_SERVOMOTOR_H
#define KITE_SERVOMOTOR_H

#include "Motor.h"

class ServoMotor: public Motor {

    float min_angle, max_anlge;

public:

    ServoMotor (int gpio, int min_pulse_width_micro_seconds, int max_pulse_width_micro_seconds, float min_angle, float max_anlge);
    void set_angle(float angle);

};


#endif //KITE_SERVOMOTOR_H
