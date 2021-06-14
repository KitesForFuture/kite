include_directories(../helpers)
include_directories(../structures)

add_library(I2cDevice
        I2cDevice.cpp
)

add_library(Bmp280Driver
        Bmp280Driver.cpp
)

add_library(Bmp280
        Bmp280.cpp
)
target_link_libraries(Bmp280
    PUBLIC Bmp280Driver
    PUBLIC I2cDevice
    PUBLIC Timer
)


add_library(Cat24c256
        Cat24c256.cpp
)
target_link_libraries(Cat24c256
  PUBLIC I2cDevice
)

add_library(Mpu6050
        Mpu6050.cpp
)
target_link_libraries(Mpu6050
  PUBLIC I2cDevice
  PUBLIC Vector3
  PUBLIC Matrix3
)
