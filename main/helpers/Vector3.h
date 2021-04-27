//
// Created by Leonard Koll on 27.04.21.
//

#ifndef KITE_VECTOR3_H
#define KITE_VECTOR3_H


class Vector3 {

    float values[3];

public:

    Vector3(float x, float y, float z);

    Vector3(float values[]);

    float &operator[](int index);

    Vector3 cross(const Vector3 &other);

    float scalar(const Vector3 &other);

    Vector3 normalize();

    Vector3 normalize(float eps);

    Vector3 operator+(Vector3 &other);

    Vector3 operator-(Vector3 &other);

    void map(float (*x_mapper)(Vector3), float (*y_mapper)(Vector3), float (*z_mapper)(Vector3));

};


#endif //KITE_VECTOR3_H
