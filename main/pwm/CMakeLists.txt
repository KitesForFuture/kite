add_library(Motor
        Motor.cpp
)

add_library(ServoMotor
        ServoMotor.cpp
        )
target_link_libraries(ServoMotor
        PUBLIC Motor
    )
