//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H

#include "Vector3.h"
#include "DataMatrix3.h"

class Matrix3 {

    float* matrix;

public:

    Matrix3(float* matrix);
    float& get (int row, int col);
    Vector3 get (int index, bool colwise);
    void normalize();
    Matrix3::multiply_cp(Matrix3 v);

};


#endif //KITE_MATRIX3_H
