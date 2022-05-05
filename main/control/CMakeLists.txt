add_library(control
 rotation_matrix.c
 controlData.c
 sensorData.c
 actuator.c
 autopilot.c)
 
 include_directories(../helpers)
target_link_libraries(control PUBLIC helpers)
