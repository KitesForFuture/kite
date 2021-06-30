//
// Created by Leonard Koll on 14.06.21.
//

#include "EightController.h"


EightController::EightController(array<float, 3> normalized_gravitation) : FlightController(normalized_gravitation) {

}

ControlParameters
EightController::get_control_parameters(array<float, 9> &position_matrix, array<float, 3> &gyro, float height,
                                        float height_derivative, float elapsed_sec) {
    return ControlParameters();
}

bool EightController::is_done() {
    // ToDo
    return false;
}