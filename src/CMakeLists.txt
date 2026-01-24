cmake_minimum_required(VERSION 3.15)

project(test_phy_engine LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

include_directories(${CMAKE_SOURCE_DIR}/../include)
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
