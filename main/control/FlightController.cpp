//
// Created by Leonard Koll on 14.06.21.
//

#include "FlightController.h"

#include <tgmath.h>

FlightController::FlightController(array<float, 3> normalized_gravitation) : normalized_gravitation{normalized_gravitation} {}

float FlightController::safe_acos(float x) {
    return fabs(x) < 1 ? acos(x) : 0;
}
