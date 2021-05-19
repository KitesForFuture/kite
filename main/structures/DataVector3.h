//
// Created by Leonard Koll on 19.05.21.
//

#ifndef KITE_DATAVECTOR3_H
#define KITE_DATAVECTOR3_H

#include "Vector3.h"


class DataVector3: public Vector3 {

    float values[3];

public:

    DataVector3(float x, float y, float z);

};


#endif //KITE_DATAVECTOR3_H
