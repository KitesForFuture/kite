//
// Created by Leonard Koll on 19.05.21.
//

#include "DataMatrix3.h"

DataMatrix3::DataMatrix3(float r1c1, float r1c2, float r1c3, float r2c1, float r2c2, float r2c3, float r3c1, float r3c2,
                         float r3c3) : Matrix3{values}, values{r1c1, r1c2, r1c3, r2c1, r2c2, r2c3, r3c1, r3c2, r3c3} {}
