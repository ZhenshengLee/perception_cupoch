cmake_minimum_required(VERSION 3.5.0)
project(cupoch_conversions)

# Add this repository's cmake modules to CMAKE_MODULE_PATH
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

include(ga_build_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
)

find_package(thrust REQUIRED QUIET)
add_definitions(${EIGEN3_DEFINITIONS})

message(STATUS "zs: CUPOCH_INCLUDE_DIRS = " ${CUPOCH_INCLUDE_DIRS})
message(STATUS "zs: CUPOCH_LIBRARIES = " ${CUPOCH_LIBRARIES})
message(STATUS "zs: EIGEN3_INCLUDE_DIRS = " ${EIGEN3_INCLUDE_DIRS})

GA_CHECK_CUDA()

catkin_package(
    INCLUDE_DIRS include
    INCLUDE_DIRS ${3RDPARTY_INCLUDE_DIRS}
    LIBRARIES cupoch_conversions
    LIBRARIES
        ${3RDPARTY_LIBRARIES}
    CATKIN_DEPENDS roscpp sensor_msgs
    DEPENDS EIGEN3
    CFG_EXTRAS
        ga_build_common.cmake
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CUPOCH_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

GA_CHECK_CUDA()

include_directories(
    .
    include
    ${CUPOCH_INCLUDE_DIRS}
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
add_dependencies(cupoch_conversions ${cupoch_conversions_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(cupoch_conversions PRIVATE
    ${CUPOCH_INCLUDE_DIRS}
)
target_link_directories(cupoch_conversions PRIVATE
    ${CUPOCH_LIBRARY_DIRS}
)
target_link_libraries(cupoch_conversions
    ${catkin_LIBRARIES}
    ${CUPOCH_LIBRARIES}
)

# Tests
if(GA_BUILD_TEST)
    add_subdirectory(test)
    add_subdirectory(tool)
endif()

# benchmark
find_package(benchmark)
if(benchmark_FOUND)
add_subdirectory(benchmarks)
endif()

# Install
install(TARGETS cupoch_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/cupoch_conversions/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
