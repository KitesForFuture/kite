//
// Created by Leonard Koll on 19.05.21.
//

#include "Matrix3.h"
#include <math.h>

Matrix3::Matrix3(float *matrix) : matrix{matrix} {}

float& Matrix3::get (int row, int col) {
    return matrix[3*row + col];
}

Vector3 Matrix3::get (int index, bool colwise) {
    Vector3 to_return {nullptr, nullptr, nullptr};
    if (colwise) {
        to_return = {
            &matrix[index],
            &matrix[3+index],
            &matrix[6+index]
        };
    } else {
        to_return = {
            &matrix[3*index],
            &matrix[3*index+1],
            &matrix[3*index+2]
        };
    }
    return to_return;
}

void Matrix3::normalize() {

    // Gram-Schmidt orthogonalization
    // (direction of first column stays constant and always only the latter two are rotated)

    float norm = sqrt(matrix[0] * matrix[0] + matrix[1] * matrix[1] + matrix[2] * matrix[2]);
    for (int i = 0; i < 3; i++) {
        matrix[i] /= norm;
    }

    float scalarProd = matrix[0] * matrix[3] + matrix[1] * matrix[4] + matrix[2] * matrix[5];
    for (int i = 0; i < 3; i++) {
        matrix[3 + i] -= scalarProd * matrix[i];
    }

    norm = sqrt(matrix[3] * matrix[3] + matrix[4] * matrix[4] + matrix[5] * matrix[5]);
    for (int i = 0; i < 3; i++) {
        matrix[3 + i] /= norm;
    }

    scalarProd = matrix[0] * matrix[6] + matrix[1] * matrix[7] + matrix[2] * matrix[8];
    for (int i = 0; i < 3; i++) {
        matrix[6 + i] -= scalarProd * matrix[i];
    }

    scalarProd = matrix[3] * matrix[6] + matrix[4] * matrix[7] + matrix[5] * matrix[8];
    for (int i = 0; i < 3; i++) {
        matrix[6 + i] -= scalarProd * matrix[3 + i];
    }

    norm = sqrt(matrix[6] * matrix[6] + matrix[7] * matrix[7] + matrix[8] * matrix[8]);
    for (int i = 0; i < 3; i++) {
        matrix[6 + i] /= norm;
    }
}
