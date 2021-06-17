//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_VECTOR3_H
#define KITE_VECTOR3_H

#include <array>

using namespace std;

class Vector3 {
public:
    
    static float get (array<float, 3>& v, int index);
    static float get_norm(array<float, 3>& v);
    static float normalize(array<float, 3>& v);
    static float normalize(array<float, 3>& v, float epsilon);
    static array<float, 3> multiply (array<float, 3>& v, float factor);
    static array<float, 3> subtract (array<float, 3>& v1, array<float, 3>& v2);
    static array<float, 3> add (array<float, 3>& v1, array<float, 3>& v2);
    static array<float, 3> cross_product(array<float, 3>& v1, array<float, 3>& v2);
    static float scalar_product(array<float, 3>& v1, array<float, 3>& v2);

};


#endif //KITE_VECTOR3_H
