#include "kitemath.h"

// acos function continuously extended beyond -1 and 1.
float safe_acos(float number_more_or_less_between_one_and_minus_one) {
    return (fabs(number_more_or_less_between_one_and_minus_one) < 1) ? acos(
            number_more_or_less_between_one_and_minus_one) : 0;
    return 0;
}

int smallpow(int x, int p) {
    int ret = 1;
    for (int i = 0; i < p; i++) {
        ret *= x;
    }
    return ret;
}

void crossProduct(float a, float b, float c, float x, float y, float z, float result[]) {
    result[0] = b * z - c * y;
    result[1] = c * x - a * z;
    result[2] = a * y - b * x;
}

float sign(float x) {
    if (x < 0)
        return -1;
    else
        return 1;
}

void mat_mult(float a[], float b[], float out[]) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[3 * i + j] = 0;
            for (int k = 0; k < 3; k++) {
                out[3 * i + j] += a[3 * i + k] * b[3 * k + j];
            }
        }
    }
}

void mat_mult_mat_transp(float a[], float b[], float out[]) {

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[3 * i + j] = 0;
            for (int k = 0; k < 3; k++) {
                out[3 * i + j] += a[3 * i + k] * b[3 * j + k];
            }
        }
    }
}

void mat_transp_mult_vec(float mat[], float a, float b, float c, float out[]) {

    for (int i = 0; i < 3; i++) {
        out[i] = mat[i] * a + mat[i + 3] * b + mat[i + 6] * c;

    }
}

void mat_mult_vec(float mat[], float a, float b, float c, float out[]) {

    for (int i = 0; i < 3; i++) {
        out[i] = mat[i * 3] * a + mat[i * 3 + 1] * b + mat[i * 3 + 2] * c;

    }
}

void normalize_matrix(float a[]) {

    // Gram-Schmidt orthogonalization
    // (direction of first column stays constant and always only the latter two are rotated)

    float norm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    for (int i = 0; i < 3; i++) {
        a[i] /= norm;
    }

    float scalarProd = a[0] * a[3] + a[1] * a[4] + a[2] * a[5];
    for (int i = 0; i < 3; i++) {
        a[3 + i] -= scalarProd * a[i];
    }

    norm = sqrt(a[3] * a[3] + a[4] * a[4] + a[5] * a[5]);
    for (int i = 0; i < 3; i++) {
        a[3 + i] /= norm;
    }

    scalarProd = a[0] * a[6] + a[1] * a[7] + a[2] * a[8];
    for (int i = 0; i < 3; i++) {
        a[6 + i] -= scalarProd * a[i];
    }

    scalarProd = a[3] * a[6] + a[4] * a[7] + a[5] * a[8];
    for (int i = 0; i < 3; i++) {
        a[6 + i] -= scalarProd * a[3 + i];
    }

    norm = sqrt(a[6] * a[6] + a[7] * a[7] + a[8] * a[8]);
    for (int i = 0; i < 3; i++) {
        a[6 + i] /= norm;
    }
}


// FUNCTIONS NECESSARY FOR REINFORCEMENT LEARNING

// scalar product of two vectors
float scalarProductOfMatrices(float A[], float B[], int length) {
    float ret = 0;
    for (int i = 0; i < length; i++) {
        ret += A[i] * B[i];
    }
    return ret;
}

float normalize(float a[], int length) {
    float norm = sqrt(scalarProductOfMatrices(a, a, length));
    if (norm > 0.00001) {
        for (int i = 0; i < length; i++) {
            a[i] = (1.0 / norm) * a[i];
        }
    }

    return norm;
}
