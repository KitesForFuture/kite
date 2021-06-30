//
// Created by Leonard Koll on 29.06.21.
//

#include "ServoMotor.h"

ServoMotor::ServoMotor(int gpio, int min_pulse_width_micro_seconds, int max_pulse_width_micro_seconds, float min_angle,
                       float max_anlge) :
                       Motor(gpio, min_pulse_width_micro_seconds, max_pulse_width_micro_seconds),
                       min_angle{min_angle}, max_anlge{max_anlge}
{}

void ServoMotor::set_angle(float angle) {
    set((angle-min_angle)/(max_anlge-min_angle));
}
