idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

add_subdirectory(helpers)

add_subdirectory(i2c)
target_link_libraries(${COMPONENT_LIB} PUBLIC bmp280)
target_link_libraries(${COMPONENT_LIB} PUBLIC cat24c256)
#target_link_libraries(${COMPONENT_LIB} PUBLIC mpu6050)

add_subdirectory(control)
target_link_libraries(${COMPONENT_LIB} PUBLIC control)

#[[
add_subdirectory(pwm)
target_link_libraries(${COMPONENT_LIB} PUBLIC pwm)
]]
