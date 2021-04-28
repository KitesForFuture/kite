//
// Created by Leonard Koll on 27.04.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H

#include "vector3.h"

class Matrix3 {

    Vector3 vectors[3];

public:

    Matrix3(float v0x, float v0y, float v0z, float v1x, float v1y, float v1z, float v2x, float v2y, float v2z);
    Matrix3(float values[]);
    Matrix3();
    Vector3& operator[] (int index);
    Matrix3 operator* (Matrix3& other);
    Matrix3 transposed_multiply (Matrix3& other);
    Vector3 transposed_multiply (Vector3& vector);
    void normalize();

};


#endif //KITE_MATRIX3_H
