cmake_minimum_required(VERSION 3.15)

project(test_phy_engine LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Respect user-provided build type (e.g. -DCMAKE_BUILD_TYPE=Release).
# If not set (single-config generators like Ninja/Make), default to Release.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

include_directories(${CMAKE_SOURCE_DIR}/../include)

option(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH "Enable CUDA acceleration for pe_synth (Clang CUDA, no nvcc)" OFF)
set(PHY_ENGINE_CUDA_PATH "/usr/local/cuda" CACHE PATH "CUDA toolkit root path (for Clang CUDA)")
set(PHY_ENGINE_CUDA_PE_SYNTH_ARCH "sm_70" CACHE STRING "CUDA GPU arch for pe_synth CUDA backend (e.g. sm_70)")
option(PHY_ENGINE_USE_LEVELDB "Enable vendored LevelDB (include/leveldb)" ON)
if(PHY_ENGINE_USE_LEVELDB)
    # Prevent pulling LevelDB's own tests/benchmarks when used as a subproject.
    set(LEVELDB_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(LEVELDB_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(LEVELDB_INSTALL OFF CACHE BOOL "" FORCE)

    # LevelDB public headers live in include/leveldb/include/leveldb/.
    include_directories(${CMAKE_SOURCE_DIR}/../include/leveldb/include)

    add_subdirectory(${CMAKE_SOURCE_DIR}/../include/leveldb ${CMAKE_BINARY_DIR}/leveldb EXCLUDE_FROM_ALL)

    # LevelDB enables `-Werror` for its own build when Clang thread-safety is available,
    # but it is added as a PUBLIC option and would leak into Phy-Engine targets.
    if(TARGET leveldb)
        get_target_property(_leveldb_iface_opts leveldb INTERFACE_COMPILE_OPTIONS)
        if(_leveldb_iface_opts AND NOT _leveldb_iface_opts STREQUAL "INTERFACE_COMPILE_OPTIONS-NOTFOUND")
            list(REMOVE_ITEM _leveldb_iface_opts -Werror -Wthread-safety)
            set_property(TARGET leveldb PROPERTY INTERFACE_COMPILE_OPTIONS "${_leveldb_iface_opts}")
        endif()
    endif()
endif()

# Keep Phy-Engine compile flags from leaking into vendored deps (like LevelDB).
if (MSVC)
    add_compile_options(/nologo /Zc:preprocessor /utf-8 /DNOMINMAX /bigobj /GR-)
else()
    add_compile_options(-Wall -Wextra -Wno-braced-scalar-init -fno-unwind-tables)
    if (WIN32)
        add_compile_options(-finput-charset=utf-8 -fexec-charset=utf-8)
    endif()
endif()
if (WIN32)
    link_libraries(ntdll)
endif()

# 添加这一行来编译phyengine.dll
add_library(phyengine SHARED dll_main.cpp)
if(PHY_ENGINE_USE_LEVELDB)
    # Allow linking LevelDB into shared libs on platforms that require PIC.
    set_property(TARGET leveldb PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_link_libraries(phyengine PRIVATE leveldb)
endif()

# verilog -> PE netlist -> PhysicsLab .sav (with IO auto-placement)
add_executable(verilog2plsav verilog2plsav.cpp)

# verilog -> PENL (LevelDB-backed) export
if(PHY_ENGINE_USE_LEVELDB)
    add_executable(verilog2penl verilog2penl.cpp)
    target_link_libraries(verilog2penl PRIVATE leveldb)
endif()

if(PHY_ENGINE_ENABLE_CUDA_PE_SYNTH)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "PHY_ENGINE_ENABLE_CUDA_PE_SYNTH requires Clang as C++ compiler (set -DCMAKE_CXX_COMPILER=clang++).")
    endif()

    if(NOT EXISTS "${PHY_ENGINE_CUDA_PATH}/include/cuda_runtime.h")
        message(FATAL_ERROR "CUDA headers not found at ${PHY_ENGINE_CUDA_PATH}/include (set -DPHY_ENGINE_CUDA_PATH=/usr/local/cuda).")
    endif()

    set(_phy_cuda_lib_dirs
        "${PHY_ENGINE_CUDA_PATH}/lib64"
        "${PHY_ENGINE_CUDA_PATH}/targets/x86_64-linux/lib"
    )
    find_library(PHY_ENGINE_CUDART_LIBRARY cudart HINTS ${_phy_cuda_lib_dirs} REQUIRED)

    set(_phy_pe_synth_cuda_src "${CMAKE_SOURCE_DIR}/pe_synth_cuda_u64_cones.cu")
    set_source_files_properties("${_phy_pe_synth_cuda_src}" PROPERTIES LANGUAGE CXX)
    add_library(pe_synth_cuda STATIC "${_phy_pe_synth_cuda_src}")
    # This static library is linked into libphyengine.so; ensure its objects are built with -fPIC.
    set_property(TARGET pe_synth_cuda PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_include_directories(pe_synth_cuda PRIVATE "${CMAKE_SOURCE_DIR}/../include" "${PHY_ENGINE_CUDA_PATH}/include")
    target_compile_options(pe_synth_cuda PRIVATE -x cuda "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_PE_SYNTH_ARCH}")
    target_link_options(pe_synth_cuda INTERFACE "--cuda-path=${PHY_ENGINE_CUDA_PATH}" "--cuda-gpu-arch=${PHY_ENGINE_CUDA_PE_SYNTH_ARCH}")
    target_link_libraries(pe_synth_cuda PUBLIC ${PHY_ENGINE_CUDART_LIBRARY})

    target_compile_definitions(pe_synth_cuda PRIVATE PHY_ENGINE_ENABLE_CUDA_PE_SYNTH=1)
    target_compile_definitions(verilog2plsav PRIVATE PHY_ENGINE_ENABLE_CUDA_PE_SYNTH=1)
    target_link_libraries(verilog2plsav PRIVATE pe_synth_cuda)
    if(PHY_ENGINE_USE_LEVELDB)
        target_compile_definitions(verilog2penl PRIVATE PHY_ENGINE_ENABLE_CUDA_PE_SYNTH=1)
        target_link_libraries(verilog2penl PRIVATE pe_synth_cuda)
    endif()
    target_compile_definitions(phyengine PRIVATE PHY_ENGINE_ENABLE_CUDA_PE_SYNTH=1)
    target_link_libraries(phyengine PRIVATE pe_synth_cuda)

    # Allow running without setting LD_LIBRARY_PATH for CUDA libs.
    foreach(_dir IN LISTS _phy_cuda_lib_dirs)
        if(EXISTS "${_dir}")
            target_link_options(verilog2plsav PRIVATE "-Wl,-rpath,${_dir}")
            if(PHY_ENGINE_USE_LEVELDB)
                target_link_options(verilog2penl PRIVATE "-Wl,-rpath,${_dir}")
            endif()
            target_link_options(phyengine PRIVATE "-Wl,-rpath,${_dir}")
        endif()
    endforeach()
endif()
