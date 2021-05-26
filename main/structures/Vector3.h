//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_VECTOR3_H
#define KITE_VECTOR3_H

#include <array>

using namespace std;

class Vector3 {

    float* value_ptrs[3];

public:

    Vector3(float* x_ptr, float* y_ptr, float* z_ptr);
    Vector3(std::array<float, 3>* values);
    float& get (int index);
    float get_norm();
    float normalize();
    float normalize(float epsilon);
    std::array<float, 3> multiply (float factor);
    std::array<float, 3> subtract (Vector3 v);
    std::array<float, 3> cross_product(Vector3 v);
    float scalar_product(Vector3 v);
    void set (std::array<float, 3> values);

};


#endif //KITE_VECTOR3_H
