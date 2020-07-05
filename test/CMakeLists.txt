macro(add_check_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Boost_ATOMIC_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_check_test)

include_directories ("${PROJECT_SOURCE_DIR}/src/")


file(GLOB_RECURSE voxelizer_SRC ${PROJECT_SOURCE_DIR}/src/timer.cpp ${PROJECT_SOURCE_DIR}/src/timer.h ${PROJECT_SOURCE_DIR}/src/voxelizer.cpp ${PROJECT_SOURCE_DIR}/src/voxelizer.h ${PROJECT_SOURCE_DIR}/src/thread_pool.cpp ${PROJECT_SOURCE_DIR}/src/thread_pool.h ${PROJECT_SOURCE_DIR}/src/commons.cpp ../src/commons.h ${PROJECT_SOURCE_DIR}/src/tri_box.cpp ${PROJECT_SOURCE_DIR}/src/tri_box.h)

add_check_test(collision_checker collision_checker.cpp collision_checker.h ${voxelizer_SRC})
add_check_test(read_rawvox read_rawvox.cpp ${voxelizer_SRC})

