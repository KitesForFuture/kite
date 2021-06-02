idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS ".")

add_subdirectory(structures)
target_link_libraries(${COMPONENT_LIB} PUBLIC vector3)
target_link_libraries(${COMPONENT_LIB} PUBLIC matrix3)

add_subdirectory(data)
target_link_libraries(${COMPONENT_LIB} PUBLIC flydata)

add_subdirectory(helpers)
target_link_libraries(${COMPONENT_LIB} PUBLIC wifi)

add_subdirectory(i2c)
target_link_libraries(${COMPONENT_LIB} PUBLIC bmp280)
target_link_libraries(${COMPONENT_LIB} PUBLIC cat24c256)
target_link_libraries(${COMPONENT_LIB} PUBLIC mpu6050)

add_subdirectory(control)
target_link_libraries(${COMPONENT_LIB} PUBLIC rotation_matrix)

add_subdirectory(pwm)
target_link_libraries(${COMPONENT_LIB} PUBLIC pwm)
