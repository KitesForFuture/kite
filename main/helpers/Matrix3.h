//
// Created by Leonard Koll on 27.04.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H


class Matrix3 {

    Vector3 vectors[3];

public:

    Matrix3(float v0x, float v0y, float v0z, float v1x, float v1y, float v1z, float v2x, float v2y, float v2z);
    Matrix3(float values[]);
    Matrix3();
    Matrix3& operator[] (int index);
    Matrix3 operator* (const Matrix3& other);
    Matrix3 transposed_multiply (const Matrix3& other);
    Vector3 transposed_multiply (const Vector3& vector);
    void normalize();

};


#endif //KITE_MATRIX3_H