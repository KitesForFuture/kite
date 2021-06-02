//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_MATRIX3_H
#define KITE_MATRIX3_H

#include <array>

using namespace std;

class Matrix3 {
public:

    static float get (array<float, 9>& m, int row, int col);
    static void set (array<float, 9>& m, int row, int col, float value);
    static array<float, 3> get (array<float, 9>& m, int index, bool colwise);
    static void set (array<float, 9>& m, int index, bool colwise, array<float, 3>& v);
    static void normalize(array<float, 9>& m);
    static array<float, 9> multiply(array<float, 9>& m1, array<float, 9>& m2);
    static array<float, 9> transpose_right_multiply(array<float, 9>& m1, array<float, 9>& m2);
    static array<float, 3> transpose_multiply(array<float, 9>& m, array<float, 3>& v);

};

#endif //KITE_MATRIX3_H
