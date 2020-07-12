macro(add_check_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    voxelizer_lib)
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_check_test)

include_directories ("${PROJECT_SOURCE_DIR}/src/")

add_check_test(collision_checker collision_checker.cpp collision_checker.h)
add_check_test(read_rawvox read_rawvox.cpp)

