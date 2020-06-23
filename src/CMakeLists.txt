#add_executable(general_test general_test.cpp)
#target_link_libraries(general_test
#    ${FCL_LIBRARIES}
#    ${Boost_SYSTEM_LIBRARY}
#    ${Boost_THREAD_LIBRARY}
#    ${Boost_DATE_TIME_LIBRARY}
#    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
#)

#add_executable(STLImportTest STLImportTest.cpp)
#target_link_libraries(STLImportTest
#	${ASSIMP_LIBRARIES}
#    ${FCL_LIBRARIES}
#    ${Boost_SYSTEM_LIBRARY}
#    ${Boost_THREAD_LIBRARY}
#    ${Boost_DATE_TIME_LIBRARY}
#    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
#)

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
)
#add_library(${PROJECT_NAME} STATIC ${voxelizer_SRC})

#add_executable(collision_checker collision_checker.cpp collision_checker.h timer.cpp timer.h voxelizer.cpp voxelizer.h thread_pool.cpp thread_pool.h commons.cpp commons.h tri_box.cpp tri_box.h)
#target_link_libraries(collision_checker
#    ${ASSIMP_LIBRARIES}
#    ${FCL_LIBRARIES}
#    ${Boost_ATOMIC_LIBRARY}
#    ${Boost_SYSTEM_LIBRARY}
#    ${Boost_THREAD_LIBRARY}
#    ${Boost_DATE_TIME_LIBRARY}
#    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
#)
