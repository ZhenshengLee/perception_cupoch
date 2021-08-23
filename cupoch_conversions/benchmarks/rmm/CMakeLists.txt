find_package(benchmark REQUIRED)

# option(PER_THREAD_DEFAULT_STREAM "Build with per-thread default stream" OFF)
# if(PER_THREAD_DEFAULT_STREAM)
#   message(STATUS "RMM: Building benchmarks with per-thread default stream")
# endif(PER_THREAD_DEFAULT_STREAM)
set(PER_THREAD_DEFAULT_STREAM ON)

# This function takes in a benchmark name and benchmark source and handles setting all of the
# associated properties and linking to build the benchmark
function(ConfigureBench BENCH_NAME BENCH_SRC)
  cuda_add_executable(${BENCH_NAME} ${BENCH_SRC}
                               "${CMAKE_CURRENT_SOURCE_DIR}/synchronization/synchronization.cpp")
  target_include_directories(${BENCH_NAME} PRIVATE
    ${CUPOCH_INCLUDE_DIRS}
    ${THRUST_INCLUDE_DIR}
    ${3RDPARTY_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )
  target_link_directories(${BENCH_NAME} PUBLIC
    ${CUPOCH_LIBRARY_DIRS}
  )
  set_target_properties(
    ${BENCH_NAME}
    PROPERTIES POSITION_INDEPENDENT_CODE ON
  )
  target_link_libraries(${BENCH_NAME} benchmark::benchmark pthread
    ${CUPOCH_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )
  target_compile_definitions(${BENCH_NAME}
                             PUBLIC "SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_${RMM_LOGGING_LEVEL}"
                             ${CUPOCH_DEFINITIONS}
  )

  if(PER_THREAD_DEFAULT_STREAM)
    target_compile_definitions(${BENCH_NAME} PUBLIC CUDA_API_PER_THREAD_DEFAULT_STREAM)
  endif(PER_THREAD_DEFAULT_STREAM)

  target_compile_options(${BENCH_NAME} PUBLIC $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall
                                              -Wno-error=deprecated-declarations>)
  if(DISABLE_DEPRECATION_WARNING)
    target_compile_options(
      ${BENCH_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-deprecated-declarations>)
    target_compile_options(${BENCH_NAME}
                           PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>)
  endif(DISABLE_DEPRECATION_WARNING)

endfunction(ConfigureBench)

# benchmark sources

# random allocations benchmark

set(RANDOM_ALLOCATIONS_BENCH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/random_allocations/random_allocations.cpp")

ConfigureBench(RANDOM_ALLOCATIONS_BENCH "${RANDOM_ALLOCATIONS_BENCH_SRC}")
