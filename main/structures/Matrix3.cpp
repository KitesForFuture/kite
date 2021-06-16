//
// Created by Leonard Koll on 19.05.21.
//

#include "Matrix3.h"
#include <math.h>
#include <iterator>

float Matrix3::get (array<float, 9>& m, int row, int col) {
    return m[3*row + col];
}

void Matrix3::set (array<float, 9>& m, int row, int col, float value) {
    m[3*row + col] = value;
}

array<float, 3> Matrix3::get (array<float, 9>& m, int index, bool colwise) {
    if (colwise) {
        return array<float, 3> { m[index],m[3+index], m[6+index]};
    } else {
        return array<float, 3> { m[3*index],m[3*index+1], m[3*index+2]};
    }
}

void Matrix3::set (array<float, 9>& m, int index, bool colwise, array<float, 3>& v) {
    if (colwise) {
        m[index] = v[0];
        m[3+index] = v[1];
        m[6+index] = v[2];
    } else {
        m[3*index] = v[0];
        m[3*index+1] = v[1];
        m[3*index+2] = v[2];
    }
}

void Matrix3::normalize(array<float, 9>& m) {

    // Gram-Schmidt orthogonalization
    // (direction of first column stays constant and always only the latter two are rotated)

    float norm = sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    for (int i = 0; i < 3; i++) {
        m[i] /= norm;
    }

    float scalarProd = m[0] * m[3] + m[1] * m[4] + m[2] * m[5];
    for (int i = 0; i < 3; i++) {
        m[3 + i] -= scalarProd * m[i];
    }

    norm = sqrt(m[3] * m[3] + m[4] * m[4] + m[5] * m[5]);
    for (int i = 0; i < 3; i++) {
        m[3 + i] /= norm;
    }

    scalarProd = m[0] * m[6] + m[1] * m[7] + m[2] * m[8];
    for (int i = 0; i < 3; i++) {
        m[6 + i] -= scalarProd * m[i];
    }

    scalarProd = m[3] * m[6] + m[4] * m[7] + m[5] * m[8];
    for (int i = 0; i < 3; i++) {
        m[6 + i] -= scalarProd * m[3 + i];
    }

    norm = sqrt(m[6] * m[6] + m[7] * m[7] + m[8] * m[8]);
    for (int i = 0; i < 3; i++) {
        m[6 + i] /= norm;
    }
}

array<float, 9> Matrix3::multiply(array<float, 9>& m1, array<float, 9>& m2) {
    array<float, 9> out {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                out[3 * i + j] += m1[3 * i + k] * m2[3 * k + j];
            }
        }
    }
    return out;
}

array<float, 3> Matrix3::multiply(array<float, 9>& m, array<float, 3>& v) {
    array<float, 3> out {0};
    for (int i = 0; i < 3; i++) {
        out[i] = m[3*i + 0] * v[0] + m[3*i + 1] * v[1] + m[3*i + 2] * v[2];
    }
    return out;
}

array<float, 9> Matrix3::transpose_right_multiply(array<float, 9>& m1, array<float, 9>& m2) {
    array<float, 9> out {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                out[3 * i + j] += m1[3 * i + k] * m2[3 * j + k];
            }
        }
    }
    return out;
}

array<float, 3> Matrix3::transpose_multiply(array<float, 9>& m, array<float, 3>& v) {
    array<float, 3> out {0};
    for (int i = 0; i < 3; i++) {
        out[i] = m[i] * v[0] + m[i + 3] * v[1] + m[i + 6] * v[2];
    }
    return out;
}
