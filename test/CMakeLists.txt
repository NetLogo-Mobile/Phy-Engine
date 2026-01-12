cmake_minimum_required(VERSION 3.15)

project(test_phy_engine LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)
enable_testing()

option(PHY_ENGINE_CUDA_CLANG_TEST "Build CUDA smoke test with Clang (no nvcc)" OFF)
set(PHY_ENGINE_CUDA_PATH "/usr/local/cuda" CACHE PATH "CUDA toolkit root path (for Clang CUDA)")
set(PHY_ENGINE_CUDA_SMOKE_ARCH "sm_70" CACHE STRING "CUDA GPU arch for smoke test (e.g. sm_70)")

option(PHY_ENGINE_BUILD_FUZZERS "Build LLVM libFuzzer targets" OFF)
set(PHY_ENGINE_FUZZ_SANITIZERS "fuzzer,address,undefined" CACHE STRING "Clang -fsanitize=... list for fuzz targets")

if (MSVC)
    add_compile_options(/nologo /Zc:preprocessor /utf-8 /DNOMINMAX /bigobj /GR-)
else()
    add_compile_options(-Wall -Wextra -Wno-braced-scalar-init -fno-unwind-tables)
    if (WIN32)
        add_compile_options(-finput-charset=utf-8 -fexec-charset=utf-8)
    endif()
endif()

file(GLOB_RECURSE test_srcs CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
     "${CMAKE_CURRENT_LIST_DIR}/*.cc"
     "${CMAKE_CURRENT_LIST_DIR}/*.cxx")

include_directories(${CMAKE_SOURCE_DIR}/../include)
if (WIN32)
    link_libraries(ntdll)
endif()

foreach(a_test IN LISTS test_srcs)
    get_filename_component(filename ${a_test} NAME_WE)
    get_filename_component(absdir ${a_test} DIRECTORY)
    get_filename_component(dir_name ${absdir} NAME)
    set(output_dir "${CMAKE_BINARY_DIR}/${dir_name}")
    file(MAKE_DIRECTORY "${output_dir}")
    add_executable(${filename} ${a_test})
    set_target_properties(${filename} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${output_dir}")
    add_test(NAME ${filename} COMMAND "${output_dir}/${filename}")
endforeach()

if(PHY_ENGINE_BUILD_FUZZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang")
        message(FATAL_ERROR "PHY_ENGINE_BUILD_FUZZERS requires Clang/AppleClang (set -DCMAKE_CXX_COMPILER=clang++).")
    endif()

    set(_phy_fuzz_dir "${CMAKE_SOURCE_DIR}/../fuzz")
    set(_phy_fuzz_out "${CMAKE_BINARY_DIR}/fuzz")
    file(MAKE_DIRECTORY "${_phy_fuzz_out}")

    add_executable(verilog_digital_fuzzer "${_phy_fuzz_dir}/verilog_digital_fuzzer.cpp")
    set_target_properties(verilog_digital_fuzzer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${_phy_fuzz_out}")
    target_include_directories(verilog_digital_fuzzer PRIVATE "${CMAKE_SOURCE_DIR}/../include")
    target_compile_options(verilog_digital_fuzzer PRIVATE -O1 -fno-omit-frame-pointer "-fsanitize=${PHY_ENGINE_FUZZ_SANITIZERS}")
    target_link_options(verilog_digital_fuzzer PRIVATE "-fsanitize=${PHY_ENGINE_FUZZ_SANITIZERS}")
endif()

if(PHY_ENGINE_CUDA_CLANG_TEST)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "PHY_ENGINE_CUDA_CLANG_TEST requires Clang as C++ compiler (set -DCMAKE_CXX_COMPILER=clang++).")
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

    set(_phy_cuda_smoke_src "${CMAKE_SOURCE_DIR}/0013.cuda/cuda_smoke_clang.cu")
    set_source_files_properties("${_phy_cuda_smoke_src}" PROPERTIES LANGUAGE CXX)
    add_executable(cuda_smoke_clang "${_phy_cuda_smoke_src}")
    set_target_properties(cuda_smoke_clang PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cuda")

    target_include_directories(cuda_smoke_clang PRIVATE "${CMAKE_SOURCE_DIR}/../include" "${PHY_ENGINE_CUDA_PATH}/include")
    target_compile_options(cuda_smoke_clang PRIVATE -x cuda "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_SMOKE_ARCH}")
    target_link_options(cuda_smoke_clang PRIVATE "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_SMOKE_ARCH}")
    target_link_libraries(cuda_smoke_clang PRIVATE ${PHY_ENGINE_CUDART_LIBRARY} ${PHY_ENGINE_CUSOLVER_LIBRARY} ${PHY_ENGINE_CUSPARSE_LIBRARY} ${PHY_ENGINE_CUBLAS_LIBRARY})

    # Allow running without setting LD_LIBRARY_PATH for CUDA libs.
    foreach(_dir IN LISTS _phy_cuda_lib_dirs)
        if(EXISTS "${_dir}")
            target_link_options(cuda_smoke_clang PRIVATE "-Wl,-rpath,${_dir}")
        endif()
    endforeach()

    add_test(NAME cuda_smoke_clang COMMAND "${CMAKE_BINARY_DIR}/cuda/cuda_smoke_clang")
    set_tests_properties(cuda_smoke_clang PROPERTIES SKIP_RETURN_CODE 77)

    set(_phy_cuda_correctness_src "${CMAKE_SOURCE_DIR}/0013.cuda/cuda_random_links_correctness.cu")
    set_source_files_properties("${_phy_cuda_correctness_src}" PROPERTIES LANGUAGE CXX)
    add_executable(cuda_random_links_correctness "${_phy_cuda_correctness_src}")
    set_target_properties(cuda_random_links_correctness PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/cuda")

    target_include_directories(cuda_random_links_correctness PRIVATE "${CMAKE_SOURCE_DIR}/../include" "${PHY_ENGINE_CUDA_PATH}/include")
    target_compile_options(cuda_random_links_correctness PRIVATE -x cuda "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_SMOKE_ARCH}")
    target_link_options(cuda_random_links_correctness PRIVATE "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_SMOKE_ARCH}")
    target_link_libraries(cuda_random_links_correctness PRIVATE ${PHY_ENGINE_CUDART_LIBRARY} ${PHY_ENGINE_CUSOLVER_LIBRARY} ${PHY_ENGINE_CUSPARSE_LIBRARY} ${PHY_ENGINE_CUBLAS_LIBRARY})

    foreach(_dir IN LISTS _phy_cuda_lib_dirs)
        if(EXISTS "${_dir}")
            target_link_options(cuda_random_links_correctness PRIVATE "-Wl,-rpath,${_dir}")
        endif()
    endforeach()

    add_test(NAME cuda_random_links_correctness COMMAND "${CMAKE_BINARY_DIR}/cuda/cuda_random_links_correctness")
    set_tests_properties(cuda_random_links_correctness PROPERTIES SKIP_RETURN_CODE 77)
endif()
