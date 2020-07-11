add_subdirectory(abseil-cpp)

set(voxelizer_SRC timer.cpp timer.h voxelizer.cpp voxelizer.h thread_pool.cpp thread_pool.h commons.cpp commons.h tri_box.cpp tri_box.h)
add_executable(voxelizer voxelizer_main.cpp ${voxelizer_SRC})

target_link_libraries(voxelizer
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Boost_ATOMIC_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    absl::flags
    absl::flags_parse
    absl::base
)