//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H

#include "Vector3.h"
using namespace std;

class Matrix3 {

    array<float, 9>* values;

public:

    Matrix3(array<float, 9>* values);
    float& get (int row, int col);
    Vector3 get (int index, bool colwise);
    void normalize();
    array<float, 9> multiply(Matrix3 m);
    array<float, 9> transpose_right_multiply(Matrix3 m);
    array<float, 3> transpose_multiply(Vector3 v);

};

#endif //KITE_MATRIX3_H
