//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H

#include "Vector3.h"
#include "DataVector3.h"

class Matrix3 {

    float* matrix;

public:

    Matrix3(float* matrix);
    float& get (int row, int col);
    Vector3 get (int index, bool colwise);
    void normalize();
    void multiply_ip(Matrix3 m);
    void transpose_right_multiply_ip(Matrix3 m);
    DataVector3 transpose_multiply_cp(Vector3 v);

};


#endif //KITE_MATRIX3_H
