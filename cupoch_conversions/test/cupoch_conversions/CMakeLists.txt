set(TEST_NAME cupoch_conversions_test)
cuda_add_executable(${TEST_NAME} test_cupoch_conversions.cpp)
target_include_directories(${TEST_NAME} PUBLIC
    ${CUPOCH_INCLUDE_DIRS}
    ${THRUST_INCLUDE_DIR}
    ${3RDPARTY_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)
target_link_directories(${TEST_NAME} PUBLIC
    ${CUPOCH_LIBRARY_DIRS}
)
target_link_libraries(${TEST_NAME}
    gtest::gtest
    gtest_main
    ${CUPOCH_LIBRARIES}
    cupoch_conversions
    ${catkin_LIBRARIES}
)
ga_add_tests(TARGET ${TEST_NAME} TEST_TYPE FUNCTIONAL)

