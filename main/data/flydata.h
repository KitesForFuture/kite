//
// Created by Leonard Koll on 16.05.21.
//

#ifndef KITE_FLYDATA_H
#define KITE_FLYDATA_H

#include <array>
using namespace std;

struct Flydata {

    array<float, 9> rotation_matrix;
    float height;

    // ToDo
    // Cycletime, Accel, Gyro, Calculated Mtion, Calvulasted G-Correction

};


#endif //KITE_FLYDATA_H
