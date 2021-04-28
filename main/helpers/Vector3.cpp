//
// Created by Leonard Koll on 27.04.21.
//

#include "vector3.h"
#include <math.h>

Vector3::Vector3(float x, float y, float z) : values{x,y,z} {}
Vector3::Vector3(float values[]) : values{values[0], values[1], values[2]} {}
Vector3::Vector3() : values{0,0,0} {}

float& Vector3::operator[] (int index) {
    return values[index];
}

Vector3 Vector3::cross (Vector3& other) {
    Vector3 result {
        values[1] * other[2] - values[2] * other[1],
        values[2] * other[0] - values[0] * other[2],
        values[0] * other[1] - values[1] * other[0]
    };
    return result;
}

float Vector3::scalar (Vector3& other) {
    return values[0]*other[0] + values[1]*other[1] + values[2]*other[2];
}

float Vector3::get_norm() {
    return sqrt(exp2(values[0]) + exp2(values[1]) + exp2(values[2]));
}

float Vector3::normalize() {
    float norm = get_norm();
    for (int i = 0; i < 3; i++) {
        values[i] /= norm;
    }
    return norm;
}

//ToDo Benni where to use this normalize
float Vector3::normalize(float eps) {
    float norm = get_norm();
    if (norm > eps) {
        for (int i = 0; i < 3; i++) {
            values[i] /= norm;
        }
    }
    return norm;
}

Vector3 Vector3::operator- (Vector3& other) {
    Vector3 result {
        values[0]-other[0],
        values[1]-other[1],
        values[2]-other[2]
    };
    return result;
}

Vector3 Vector3::operator+ (Vector3& other) {
    Vector3 result {
    values[0]+other[0],
    values[1]+other[1],
    values[2]+other[2]
    };
    return result;
}

Vector3 Vector3::multiply (float value) {
    Vector3 result {
    values[0]*value,
    values[1]*value,
    values[2]*value
    };
    return result;
}

void Vector3::map (float (*x_mapper)(Vector3), float (*y_mapper)(Vector3), float (*z_mapper)(Vector3)) {
    values[0] = x_mapper(values);
    values[1] = y_mapper(values);
    values[2] = z_mapper(values);
}
