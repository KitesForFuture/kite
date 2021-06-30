include_directories(../helpers)
include_directories(../i2c)

add_library(Position
        Position.cpp
)
target_link_libraries(Position
    PUBLIC Timer
    PUBLIC Matrix3
    PUBLIC Vector3
)

add_library(FlightController
        FlightController.cpp
)

add_library(HoverController
        HoverController.cpp
)
target_link_libraries(HoverController
        PUBLIC FlightController
    )

add_library(EightController
        EightController.cpp
)

add_library(LandingController
        LandingController.cpp
)

add_library(ManualController
        ManualController.cpp
)

add_library(StateMachine
        StateMachine.cpp
)
