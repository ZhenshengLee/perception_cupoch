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

find_package(PCL QUIET)
if(PCL_FOUND)
    set(TOOL_NAME pcl_conversions_test_node)
    add_executable(${TOOL_NAME} pcl_conversions_test_node.cpp)
    target_include_directories(${TOOL_NAME} PRIVATE
        ${PCL_INCLUDE_DIRS}
    )
    target_compile_definitions(${TOOL_NAME} PRIVATE
    )
    target_link_libraries(${TOOL_NAME}
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    add_dependencies(${TOOL_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )
    install(TARGETS ${TOOL_NAME}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
