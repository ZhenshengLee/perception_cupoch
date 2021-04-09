set(TEST_NAME cupoch_conversions_test_node)
cuda_add_executable(${TEST_NAME} cupoch_conversions_test_node.cu)
add_dependencies(${EXENAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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
