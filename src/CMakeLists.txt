add_executable(general_test general_test.cpp)
target_link_libraries(general_test
    ${FCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(STLImportTest STLImportTest.cpp)
target_link_libraries(STLImportTest
	${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(mujin mujin.cpp Timer.cpp Timer.h Voxelizer.cpp Voxelizer.h ThreadPool.cpp ThreadPool.h Commons.cpp Commons.h TriBox.cpp TriBox.h)
target_link_libraries(mujin
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Boost_ATOMIC_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)
