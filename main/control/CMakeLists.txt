include_directories(../helpers)
include_directories(../i2c)

add_library(control
 rotation_matrix.c
)
target_link_libraries(control
    PUBLIC timer
    PUBLIC mpu6050
)
