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


file(GLOB_RECURSE voxelizer_SRC ${PROJECT_SOURCE_DIR}/src/timer.cpp ${PROJECT_SOURCE_DIR}/src/timer.h ${PROJECT_SOURCE_DIR}/src/voxelizer.cpp ${PROJECT_SOURCE_DIR}/src/voxelizer.h ${PROJECT_SOURCE_DIR}/src/threadPool.cpp ${PROJECT_SOURCE_DIR}/src/threadPool.h ${PROJECT_SOURCE_DIR}/src/commons.cpp ../src/commons.h ${PROJECT_SOURCE_DIR}/src/triBox.cpp ${PROJECT_SOURCE_DIR}/src/triBox.h)

add_check_test(collisionChecker collisionChecker.cpp collisionChecker.h ${voxelizer_SRC})

