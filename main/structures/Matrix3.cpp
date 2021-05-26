//
// Created by Leonard Koll on 19.05.21.
//

#include "Matrix3.h"
#include <math.h>
#include <iterator>

Matrix3::Matrix3(array<float, 9>* values) : values{values} {}

float& Matrix3::get (int row, int col) {
    return (*values)[3*row + col];
}

Vector3 Matrix3::get (int index, bool colwise) {
    Vector3 to_return {nullptr, nullptr, nullptr};
    if (colwise) {
        to_return = {
            &((*values)[index]),
            &((*values)[3+index]),
            &((*values)[6+index])
        };
    } else {
        to_return = {
            &((*values)[3*index]),
            &((*values)[3*index+1]),
            &((*values)[3*index+2])
        };
    }
    return to_return;
}

void Matrix3::normalize() {

    // Gram-Schmidt orthogonalization
    // (direction of first column stays constant and always only the latter two are rotated)

    float norm = sqrt((*values)[0] * (*values)[0] + (*values)[1] * (*values)[1] + (*values)[2] * (*values)[2]);
    for (int i = 0; i < 3; i++) {
        (*values)[i] /= norm;
    }

    float scalarProd = (*values)[0] * (*values)[3] + (*values)[1] * (*values)[4] + (*values)[2] * (*values)[5];
    for (int i = 0; i < 3; i++) {
        (*values)[3 + i] -= scalarProd * (*values)[i];
    }

    norm = sqrt((*values)[3] * (*values)[3] + (*values)[4] * (*values)[4] + (*values)[5] * (*values)[5]);
    for (int i = 0; i < 3; i++) {
        (*values)[3 + i] /= norm;
    }

    scalarProd = (*values)[0] * (*values)[6] + (*values)[1] * (*values)[7] + (*values)[2] * (*values)[8];
    for (int i = 0; i < 3; i++) {
        (*values)[6 + i] -= scalarProd * (*values)[i];
    }

    scalarProd = (*values)[3] * (*values)[6] + (*values)[4] * (*values)[7] + (*values)[5] * (*values)[8];
    for (int i = 0; i < 3; i++) {
        (*values)[6 + i] -= scalarProd * (*values)[3 + i];
    }

    norm = sqrt((*values)[6] * (*values)[6] + (*values)[7] * (*values)[7] + (*values)[8] * (*values)[8]);
    for (int i = 0; i < 3; i++) {
        (*values)[6 + i] /= norm;
    }
}

array<float, 9> Matrix3::multiply(Matrix3 m) {
    array<float, 9> result {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                result[3 * i + j] += (*values)[3 * i + k] * (*m.values)[3 * k + j];
            }
        }
    }
    return result;
}

array<float, 9> Matrix3::transpose_right_multiply(Matrix3 m) {
    array<float, 9> result {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                result[3 * i + j] += (*values)[3 * i + k] * (*m.values)[3 * j + k];
            }
        }
    }
    return result;
}

array<float, 3> Matrix3::transpose_multiply(Vector3 v) {
    array<float, 3> out;
    for (int i = 0; i < 3; i++) {
        out[i] = (*values)[i] * v.get(0) + (*values)[i + 3] * v.get(1) + (*values)[i + 6] * v.get(2);
    }
    return out;
}
