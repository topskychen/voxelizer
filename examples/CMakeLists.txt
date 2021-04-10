include_directories ("${PROJECT_SOURCE_DIR}/src/")

add_executable(collision_checker collision_checker.cpp collision_checker.h ${voxelizer_SRC})
target_link_libraries(collision_checker
    voxelizer_lib
)

add_executable(read_rawvox read_rawvox.cpp)
target_link_libraries(read_rawvox
    voxelizer_lib
)

add_executable(read_meta read_meta.cpp)
target_link_libraries(read_meta
    voxelizer_lib
)

add_executable(check_side check_side.cpp)
target_link_libraries(check_side
    voxelizer_lib
)
