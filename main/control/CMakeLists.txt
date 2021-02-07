include_directories(../helpers)

add_library(control
 rotation_matrix.c
)
target_link_libraries(control PUBLIC timer)
