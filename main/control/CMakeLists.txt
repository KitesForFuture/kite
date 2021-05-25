include_directories(../helpers)
include_directories(../i2c)

add_library(rotation_matrix
        rotation_matrix.cpp
)
target_link_libraries(rotation_matrix
    PUBLIC timer
    PUBLIC matrix3
    PUBLIC vector3
    PUBLIC datamatrix3
    PUBLIC datavector3
)
