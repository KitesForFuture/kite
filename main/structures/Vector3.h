//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_VECTOR3_H
#define KITE_VECTOR3_H

class Vector3 {

    float* value_ptrs[3];

public:

    Vector3(float* x_ptr, float* y_ptr, float* z_ptr);
    float& get (int index);
    float get_norm();
    float normalize();
    float normalize(float epsilon);
    void multiply_ip(float factor);
    void substract_ip(Vector3 v);
    void cross_product_ip(Vector3 v);
    float scalar_product(Vector3 v);

};


#endif //KITE_VECTOR3_H
