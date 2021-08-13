cmake_minimum_required(VERSION 3.5.0)
project(cupoch_conversions)

# Add this repository's cmake modules to CMAKE_MODULE_PATH
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

include(ga_build_common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# 使用eigen from thirdparty
# find_package(Eigen3 REQUIRED QUIET)
find_package(thrust REQUIRED QUIET)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdfdom REQUIRED)

message("zs: CUPOCH_INCLUDE_DIRS = " ${CUPOCH_INCLUDE_DIRS})
message("zs: CUPOCH_LIBRARIES = " ${CUPOCH_LIBRARIES})
message("zs: EIGEN3_INCLUDE_DIRS = " ${EIGEN3_INCLUDE_DIRS})

GA_CHECK_CUDA()

include_directories(
    include
    ${CUPOCH_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY res
  DESTINATION lib/${PROJECT_NAME}/
)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    ${CUPOCH_NVCC_FLAGS}
)
add_definitions(${CUPOCH_DEFINITIONS})
message("zs: ${CMAKE_PROJECT_NAME} CUDA_NVCC_FLAGS = " ${CUDA_NVCC_FLAGS})

# C++ library
cuda_add_library(cupoch_conversions STATIC src/cupoch_conversions.cu)
ament_target_dependencies(cupoch_conversions
    rclcpp
    sensor_msgs
    urdfdom
)
target_include_directories(cupoch_conversions PRIVATE
    ${CUPOCH_INCLUDE_DIRS}
)
target_link_directories(cupoch_conversions PRIVATE
    ${CUPOCH_LIBRARY_DIRS}
)
target_link_libraries(cupoch_conversions
    ${CUPOCH_LIBRARIES}
)

# Tests
if(GA_BUILD_TEST)
    add_subdirectory(test)
    add_subdirectory(tool)
endif()

# Install
install(TARGETS cupoch_conversions
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/cupoch_conversions/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cupoch_conversions)
# ament_export_targets(export_${PROJECT_NAME})
# ament_export_interfaces(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp)

ament_package()
