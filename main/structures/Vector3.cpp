//
// Created by Leonard Koll on 19.05.21.
//

#include "Vector3.h"
#include <math.h>

float Vector3::get (array<float, 3>& v, int index) {
    return v[index];
}

float Vector3::get_norm(array<float, 3>& v) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float Vector3::normalize(array<float, 3>& v) {
    return normalize(v, 0);
}

float Vector3::normalize(array<float, 3>& v, float epsilon) {
    auto norm = get_norm(v);
    if(norm > epsilon) {
        for (int i=0; i<3; i++) {
            v[i] /= norm;
        }
    }
    return norm;
}

array<float, 3> Vector3::multiply(array<float, 3>& v, float factor) {
    return array<float, 3> {
            v[0] * factor,
            v[1] * factor,
            v[2] * factor
    };
}

array<float, 3> Vector3::subtract (array<float, 3>& v1, array<float, 3>& v2) {
    return array<float, 3> {
            v1[0] - v2[0],
            v1[1] - v2[1],
            v1[2] - v2[2]
    };
}

array<float, 3> Vector3::add (array<float, 3>& v1, array<float, 3>& v2) {
    return array<float, 3> {
            v1[0] + v2[0],
            v1[1] + v2[1],
            v1[2] + v2[2]
    };
}

array<float, 3> Vector3::cross_product(array<float, 3>& v1, array<float, 3>& v2) {
    return array<float, 3> {
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[3],
        v1[0] * v2[1] - v1[1] * v2[0]
    };
}

float Vector3::scalar_product(array<float, 3>& v1, array<float, 3>& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
