idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS ".")

add_library(Config Config.cpp)
target_link_libraries(Config PUBLIC Cat24c256)
target_link_libraries(${COMPONENT_LIB} PUBLIC Config)

add_subdirectory(structures)
target_link_libraries(${COMPONENT_LIB} PUBLIC Vector3)
target_link_libraries(${COMPONENT_LIB} PUBLIC Matrix3)

add_subdirectory(data)
target_link_libraries(${COMPONENT_LIB} PUBLIC FlyData)

add_subdirectory(helpers)
target_link_libraries(${COMPONENT_LIB} PUBLIC Wifi)

add_subdirectory(i2c)
target_link_libraries(${COMPONENT_LIB} PUBLIC Bmp280)
target_link_libraries(${COMPONENT_LIB} PUBLIC Mpu6050)

add_subdirectory(control)
target_link_libraries(${COMPONENT_LIB} PUBLIC Position)

add_subdirectory(pwm)
target_link_libraries(${COMPONENT_LIB} PUBLIC Motor)
