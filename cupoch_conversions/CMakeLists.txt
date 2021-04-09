cmake_minimum_required(VERSION 3.5.0)
project(cupoch_conversions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gpuac_base
)

# 使用eigen from thirdparty
# find_package(Eigen3 REQUIRED QUIET)
find_package(thrust REQUIRED QUIET)

message(STATUS "zs: CUPOCH_INCLUDE_DIRS = " ${CUPOCH_INCLUDE_DIRS})
message(STATUS "zs: CUPOCH_LIBRARIES = " ${CUPOCH_LIBRARIES})
message(STATUS "zs: EIGEN3_INCLUDE_DIRS = " ${EIGEN3_INCLUDE_DIRS})

GA_CHECK_CUDA()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cupoch_conversions
 CATKIN_DEPENDS roscpp sensor_msgs
 DEPENDS EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CUPOCH_INCLUDE_DIRS}
)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

# version checker is not available
if(CMAKE_CROSSCOMPILING)
    if(NOT CUDA_ARCH)
            message(FATAL_ERROR "Please define the CUDA_ARCH CMake variable")
    endif()
else()
if(${GA_ROS_VERSION} STREQUAL "melodic")
    set(CUDA_ARCH "sm_61")
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
        # xavier and xavier-nx
        set(CUDA_ARCH "sm_72")
    endif()
else()
    # sm_50 for dell lap, sm_75 for target machine
    set(CUDA_ARCH "sm_75")
    # set(CUDA_ARCH "sm_50")
endif()
    message(STATUS "zs: CUDA_ARCH = " ${CUDA_ARCH})
endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    -arch=${CUDA_ARCH}
    -std=c++14
    --ptxas-options=-v
    -O3
    -use_fast_math
    --expt-relaxed-constexpr
    --expt-extended-lambda
    --default-stream per-thread
    --use_fast_math
    -Xcudafe "--diag_suppress=integer_sign_change"
    -Xcudafe "--diag_suppress=partial_override"
    -Xcudafe "--diag_suppress=virtual_function_decl_hidden"
)
add_definitions(-DUSE_RMM)
# -G
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-G;-g)
    add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()
message(STATUS "zs: CUDA_NVCC_FLAGS = " ${CUDA_NVCC_FLAGS})

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

# Install
install(TARGETS cupoch_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/cupoch_conversions/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
