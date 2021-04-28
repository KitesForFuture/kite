//
// Created by Leonard Koll on 27.04.21.
//

#include "matrix3.h"

Matrix3::Matrix3(float r1c1, float r1c2, float r1c3, float r2c1, float r2c2, float r2c3, float r3c1, float r3c2, float r3c3)
:vectors {
    Vector3 {r1c1, r2c1, r3c1},
    Vector3 {r1c2, r2c2, r3c2},
    Vector3 {r1c3, r2c3, r3c3}
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

Matrix3 Matrix3::multiply (Matrix3& other) {
    return multiply(other, false);
}

Matrix3 Matrix3::multiply (Matrix3& other, bool transpose_right) {
    Matrix3 result;
    if (transpose_right) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i][j] += vectors[i+k][j] * other[j][i+k];
                }
            }
        }
    } else {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i][j] += vectors[i+k][j] * other[i][j+k];
                }
            }
        }
    }
    return result;
}

Vector3 Matrix3::multiply (Vector3& vector) {
    return multiply(vector, false);
}

Vector3 Matrix3::multiply (Vector3& vector, bool transpose) {
    Vector3 result;
    if (transpose) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i] += vectors[j+k][i] * vector[i+k];
                }
            }
        }
    } else {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i] += vectors[i+k][j] * vector[i+k];
                }
            }
        }
    }
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
