add_subdirectory(abseil-cpp)

set(voxelizer_SRC timer.cpp timer.h voxelizer.cpp voxelizer.h thread_pool.cpp thread_pool.h commons.cpp commons.h tri_box.cpp tri_box.h)

add_library(voxelizer_lib ${voxelizer_SRC})

target_link_libraries(voxelizer_lib
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Boost_ATOMIC_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    absl::flags
    absl::flags_parse
    absl::base
    absl::strings
    absl::time
)

add_executable(voxelizer voxelizer_main.cpp)

target_link_libraries(voxelizer
    voxelizer_lib
    absl::flags
    absl::flags_parse
)