add_executable(general_test general_test.cpp)
target_link_libraries(general_test
    ${FCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)
