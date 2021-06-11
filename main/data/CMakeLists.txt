include_directories(../i2c)
include_directories(../position)

add_library(flydata
        flydata.cpp
)
target_link_libraries(flydata
        PUBLIC mpu6050
        PUBLIC position
)
