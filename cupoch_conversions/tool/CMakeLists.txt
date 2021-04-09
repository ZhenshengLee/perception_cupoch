set(TEST_NAME cupoch_conversions_test_node)
cuda_add_executable(${TEST_NAME} cupoch_conversions_test_node.cu)
ament_target_dependencies(${TEST_NAME}
    rclcpp
    sensor_msgs
)
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
    ${CUPOCH_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    cupoch_conversions
    ${catkin_LIBRARIES}
)
