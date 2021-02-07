include_directories(../helpers)

add_library(interchip
 interchip.c
)

add_library(bmp280
 bmp280.c
)
target_link_libraries(bmp280 PUBLIC timer)

add_library(cat24c256
 cat24c256.c
)

add_library(mpu6050
 mpu6050.c
)
target_link_libraries(mpu6050 PUBLIC kitemath)