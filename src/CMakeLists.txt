cmake_minimum_required(VERSION 3.15)

project(test_phy_engine LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

if (MSVC)
    add_compile_options(/nologo /Zc:preprocessor /utf-8 /DNOMINMAX /bigobj /GR-)
else()
    add_compile_options(-Wall -Wextra -Wno-braced-scalar-init -fno-unwind-tables)
    if (WIN32)
        add_compile_options(-finput-charset=utf-8 -fexec-charset=utf-8)
    endif()
endif()

include_directories(${CMAKE_SOURCE_DIR}/../include)
if (WIN32)
    link_libraries(ntdll)
endif()

# 添加这一行来编译phyengine.dll
add_library(phyengine SHARED dll_main.cpp)

# verilog -> PE netlist -> PhysicsLab .sav (with IO auto-placement)
add_executable(verilog2plsav verilog2plsav.cpp)
