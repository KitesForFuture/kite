//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_DATAMATRIX3_H
#define KITE_DATAMATRIX3_H

#include "Matrix3.h"


class DataMatrix3: public Matrix3 {

    float values[9];

public:

    DataMatrix3(float r1c1, float r1c2, float r1c3,
                float r2c1, float r2c2, float r2c3,
                float r3c1, float r3c2, float r3c3);

};


#endif //KITE_DATAMATRIX3_H
