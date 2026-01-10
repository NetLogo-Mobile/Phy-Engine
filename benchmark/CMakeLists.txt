cmake_minimum_required(VERSION 3.15)

project(phy_engine_benchmark LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)

option(PHY_ENGINE_CUDA_CLANG_BENCH "Build CUDA benchmarks with Clang (no nvcc)" OFF)
set(PHY_ENGINE_CUDA_PATH "/usr/local/cuda" CACHE PATH "CUDA toolkit root path (for Clang CUDA)")
set(PHY_ENGINE_CUDA_BENCH_ARCH "sm_70" CACHE STRING "CUDA GPU arch for benchmarks (e.g. sm_70)")

if(MSVC)
    add_compile_options(/nologo /Zc:preprocessor /utf-8 /DNOMINMAX /bigobj /GR-)
else()
    add_compile_options(-Wall -Wextra -Wno-braced-scalar-init -fno-unwind-tables)
    if(WIN32)
        add_compile_options(-finput-charset=utf-8 -fexec-charset=utf-8)
    endif()
endif()

include_directories(${CMAKE_SOURCE_DIR}/../include)

add_executable(bench_100000_random_links_cpu "${CMAKE_SOURCE_DIR}/0001.models/100000_random_links_cpu.cpp")
set_target_properties(bench_100000_random_links_cpu PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

if(PHY_ENGINE_CUDA_CLANG_BENCH)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "PHY_ENGINE_CUDA_CLANG_BENCH requires Clang as C++ compiler (set -DCMAKE_CXX_COMPILER=clang++).")
    endif()

    if(NOT EXISTS "${PHY_ENGINE_CUDA_PATH}/include/cuda_runtime.h")
        message(FATAL_ERROR "CUDA headers not found at ${PHY_ENGINE_CUDA_PATH}/include (set -DPHY_ENGINE_CUDA_PATH=/usr/local/cuda).")
    endif()

    set(_phy_cuda_lib_dirs
        "${PHY_ENGINE_CUDA_PATH}/lib64"
        "${PHY_ENGINE_CUDA_PATH}/targets/x86_64-linux/lib"
    )
    find_library(PHY_ENGINE_CUDART_LIBRARY cudart HINTS ${_phy_cuda_lib_dirs} REQUIRED)
    find_library(PHY_ENGINE_CUSOLVER_LIBRARY cusolver HINTS ${_phy_cuda_lib_dirs} REQUIRED)
    find_library(PHY_ENGINE_CUSPARSE_LIBRARY cusparse HINTS ${_phy_cuda_lib_dirs} REQUIRED)
    find_library(PHY_ENGINE_CUBLAS_LIBRARY cublas HINTS ${_phy_cuda_lib_dirs} REQUIRED)

    function(_phy_add_cuda_bench target_name source_rel)
        set(_src "${CMAKE_SOURCE_DIR}/${source_rel}")
        set_source_files_properties("${_src}" PROPERTIES LANGUAGE CXX)
        add_executable(${target_name} "${_src}")
        set_target_properties(${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
        target_include_directories(${target_name} PRIVATE "${CMAKE_SOURCE_DIR}/../include" "${PHY_ENGINE_CUDA_PATH}/include")
        target_compile_options(${target_name} PRIVATE -x cuda "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_BENCH_ARCH}")
        target_link_options(${target_name} PRIVATE "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_BENCH_ARCH}")
        target_link_libraries(${target_name} PRIVATE ${PHY_ENGINE_CUDART_LIBRARY} ${PHY_ENGINE_CUSOLVER_LIBRARY} ${PHY_ENGINE_CUSPARSE_LIBRARY} ${PHY_ENGINE_CUBLAS_LIBRARY})
        foreach(_dir IN LISTS _phy_cuda_lib_dirs)
            if(EXISTS "${_dir}")
                target_link_options(${target_name} PRIVATE "-Wl,-rpath,${_dir}")
            endif()
        endforeach()
    endfunction()

    _phy_add_cuda_bench(bench_1000000xR_cuda_efficiency "0001.models/1000000xR_cuda_efficiency.cu")
    _phy_add_cuda_bench(bench_100000_random_links_cuda "0001.models/100000_random_links_cuda.cu")
    _phy_add_cuda_bench(bench_100000_random_links_compare "0001.models/100000_random_links_compare.cu")
endif()
