//
// Created by Leonard Koll on 27.04.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H

#include "vector3.h"

class Matrix3 {

    Vector3 vectors[3];

public:

    Matrix3(float r1c1, float r1c2, float r1c3, float r2c1, float r2c2, float r2c3, float r3c1, float r3c2, float r3c3);
    Matrix3();
    Vector3& operator[] (int index);
    Matrix3 multiply (Matrix3& other);
    Vector3 multiply (Vector3& vector);
    Matrix3 multiply (Matrix3& other, bool transpose_left, bool transpose_right);
    Vector3 multiply (Vector3& vector, bool transpose);
    void normalize();

};


#endif //KITE_MATRIX3_H
