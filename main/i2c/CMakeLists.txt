include_directories(../helpers)
include_directories(../structures)

add_library(i2cdevice
        i2cdevice.cpp
)

add_library(bmp280_driver
    bmp280_driver.cpp
)

add_library(bmp280
    bmp280.cpp
)
target_link_libraries(bmp280
    PUBLIC bmp280_driver
    PUBLIC i2cdevice
    PUBLIC timer
)


add_library(cat24c256
    cat24c256.cpp
)
target_link_libraries(cat24c256
  PUBLIC i2cdevice
)

add_library(mpu6050
        mpu6050.cpp
)
target_link_libraries(mpu6050
  PUBLIC i2cdevice
  PUBLIC vector3
  PUBLIC matrix3
)
