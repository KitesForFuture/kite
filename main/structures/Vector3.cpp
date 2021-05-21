//
// Created by Leonard Koll on 19.05.21.
//

#include "Vector3.h"
#include <math.h>

Vector3::Vector3(float *x_ptr, float *y_ptr, float *z_ptr) : value_ptrs{x_ptr, y_ptr, z_ptr} {}

float& Vector3::get (int index) {
    return *value_ptrs[index];
}

float Vector3::get_norm() {
    return sqrt(*value_ptrs[0] * *value_ptrs[0]
        + *value_ptrs[1] * *value_ptrs[1]
        + *value_ptrs[2] * *value_ptrs[2]);
}

float Vector3::normalize() {
    return normalize(0);
}

float Vector3::normalize(float epsilon) {
    auto norm = get_norm();
    if(norm > epsilon) {
        for (int i=0; i<3; i++) {
            *value_ptrs[i] /= norm;
        }
    }
    return norm;
}

void Vector3::multiply_ip(float factor) {
    for (int i=0; i<3; i++) {
        *value_ptrs[i] *= factor;
    }
}

void Vector3::substract_ip(Vector3 v) {
    for (int i=0; i<3; i++) {
        *value_ptrs[i] -= v.get(i);
    }
}
