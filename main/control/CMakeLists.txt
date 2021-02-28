include_directories(../helpers)

add_library(rotation_matrix
 rotation_matrix.c
)
target_link_libraries(rotation_matrix
    PUBLIC timer
)
