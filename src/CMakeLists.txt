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

set(voxelizer_SRC timer.cpp timer.h voxelizer.cpp voxelizer.h threadPool.cpp threadPool.h commons.cpp commons.h triBox.cpp triBox.h)
add_executable(voxelizer voxelizerMain.cpp ${voxelizer_SRC})
target_link_libraries(voxelizer
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Boost_ATOMIC_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)
#add_library(${PROJECT_NAME} STATIC ${voxelizer_SRC})

#add_executable(collisionChecker collisionChecker.cpp collisionChecker.h timer.cpp timer.h voxelizer.cpp voxelizer.h threadPool.cpp threadPool.h commons.cpp commons.h triBox.cpp triBox.h)
#target_link_libraries(collisionChecker
#    ${ASSIMP_LIBRARIES}
#    ${FCL_LIBRARIES}
#    ${Boost_ATOMIC_LIBRARY}
#    ${Boost_SYSTEM_LIBRARY}
#    ${Boost_THREAD_LIBRARY}
#    ${Boost_DATE_TIME_LIBRARY}
#    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
#)
