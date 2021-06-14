include_directories(../i2c)
include_directories(../position)

add_library(FlyData
        FlyData.cpp
)
target_link_libraries(FlyData
        PUBLIC Mpu6050
        PUBLIC Position
)
