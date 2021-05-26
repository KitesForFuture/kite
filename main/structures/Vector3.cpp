//
// Created by Leonard Koll on 19.05.21.
//

#include "Vector3.h"
#include <math.h>

Vector3::Vector3(float *x_ptr, float *y_ptr, float *z_ptr) : value_ptrs{x_ptr, y_ptr, z_ptr} {}

Vector3::Vector3(array<float, 3>* values) : value_ptrs{&((*values)[0]), &((*values)[1]), &((*values)[2])} {}

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

array<float, 3> Vector3::multiply(float factor) {
    array<float, 3> out;
    for (int i=0; i<3; i++) {
        out[i] = *value_ptrs[i] * factor;
    }
    return out;
}

array<float, 3> Vector3::subtract(Vector3 v) {
    array<float, 3> out;
    for (int i=0; i<3; i++) {
        out[i] = *value_ptrs[i] - v.get(i);
    }
    return out;
}

array<float, 3> Vector3::cross_product(Vector3 v) {
    array<float, 3> out = {
        *value_ptrs[1] * v.get(2) - *value_ptrs[2] * v.get(1),
        *value_ptrs[2] * v.get(0) - *value_ptrs[0] * v.get(3),
        *value_ptrs[0] * v.get(1) - *value_ptrs[1] * v.get(0)
    };
    return out;
}

float Vector3::scalar_product(Vector3 v) {
    return *value_ptrs[0] * v.get(0) + *value_ptrs[1] * v.get(1) + *value_ptrs[2] * v.get(2);
}

void Vector3::set(array<float, 3> values) {
    for (int i=0; i<3; i++) {
        *value_ptrs[i] = values[i];
    }
}
