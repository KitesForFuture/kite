//
// Created by Leonard Koll on 27.04.21.
//

#include "matrix3.h"

Matrix3::Matrix3(float v0x, float v0y, float v0z, float v1x, float v1y, float v1z, float v2x, float v2y, float v2z)
:vectors {
    Vector3 {v0x, v0y, v0z},
    Vector3 {v1x, v1y, v1z},
    Vector3 {v2x, v2y, v2z}
} {}

Matrix3::Matrix3(float values[])
:vectors {
    Vector3 {values[0], values[1], values[2]},
    Vector3 {values[3], values[4], values[5]},
    Vector3 {values[6], values[7], values[8]}
} {}

Matrix3::Matrix3()
:vectors {
    Vector3 {0, 0, 0},
    Vector3 {0, 0, 0},
    Vector3 {0, 0, 0}
} {}

Vector3& Matrix3::operator[] (int index) {
    return vectors[index];
}

// ToDo Review by Benni
Matrix3 Matrix3::operator* (Matrix3& other) {
    Matrix3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                result[i][j] += vectors[i][j+k] * other[i+k][j];
            }
        }
    }
    return result;
}

// ToDo Review by Benni
Matrix3 Matrix3::transposed_multiply (Matrix3& other) {
    Matrix3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                result[i][j] += vectors[i][j+k] * other[i][j+k];
            }
        }
    }
    return result;
}

Vector3 Matrix3::transposed_multiply (Vector3& vector) {
    Vector3 result {
        vectors[0][0] * vector[0] + vectors[0][1] * vector[1] + vectors[0][2] * vector[2],
        vectors[1][0] * vector[0] + vectors[1][1] * vector[1] + vectors[1][2] * vector[2],
        vectors[2][0] * vector[0] + vectors[2][1] * vector[1] + vectors[2][2] * vector[2]
    };
    return result;
}

// ToDo Review by Benni, naming?
void Matrix3::normalize() {

    // Gram-Schmidt orthogonalization
    // (direction of first column stays constant and always only the latter two are rotated)

    vectors[0].normalize();

    for (int i = 0; i < 3; i++) {
        vectors[1][i] -= vectors[0].scalar(vectors[1]) * vectors[0][i];
    }

    vectors[1].normalize();

    for (int i = 0; i < 3; i++) {
        vectors[2][i] -= vectors[0].scalar(vectors[2]) * vectors[0][i];
    }

    for (int i = 0; i < 3; i++) {
        vectors[2][i] -= vectors[1].scalar(vectors[2]) * vectors[1][i];
    }

    vectors[2].normalize();
}
