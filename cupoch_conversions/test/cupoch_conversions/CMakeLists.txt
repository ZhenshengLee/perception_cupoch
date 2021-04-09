set(TEST_NAME cupoch_conversions_test)
cuda_add_executable(${TEST_NAME} test_cupoch_conversions.cpp)
target_include_directories(${TEST_NAME} PUBLIC
    ${CUPOCH_INCLUDE_DIRS}
    ${THRUST_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)
target_link_directories(${TEST_NAME} PUBLIC
    ${CUPOCH_LIBRARY_DIRS}
)
target_link_libraries(${TEST_NAME}
    gtest_main
    gtest::gtest
    ${CUPOCH_LIBRARIES}
    cupoch_conversions
)
install(TARGETS ${TEST_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ga_add_tests(TARGET ${TEST_NAME} TEST_TYPE FUNCTIONAL)

