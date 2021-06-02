include_directories(../helpers)
include_directories(../structures)

add_library(interchip
        i2c_device.cpp
)

add_library(bmp280
        bmp280.cpp
)
target_link_libraries(bmp280 
  PUBLIC timer
  PUBLIC interchip
)

add_library(cat24c256
        cat24c256.cpp
)
target_link_libraries(cat24c256 
  PUBLIC interchip
)

add_library(mpu6050
        mpu6050.cpp
)
target_link_libraries(mpu6050
  PUBLIC interchip
  PUBLIC vector3
  PUBLIC matrix3
)
