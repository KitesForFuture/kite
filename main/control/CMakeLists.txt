include_directories(../helpers)
include_directories(../i2c)

add_library(position
        position.cpp
)
target_link_libraries(position
    PUBLIC timer
    PUBLIC matrix3
    PUBLIC vector3
)
