//
// Created by Leonard Koll on 19.05.21.
//

#include "DataVector3.h"

DataVector3::DataVector3(float x, float y, float z) : Vector3(&values[0], &values[1], &values[2]), values{x,y,z} {}
