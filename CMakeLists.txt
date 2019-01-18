cmake_minimum_required(VERSION 3.11)
project(FLUID_SIM)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(ALTLIB_DIR ${PROJECT_SOURCE_DIR}/deps/altlib)
add_subdirectory(${ALTLIB_DIR})

set(ALTMATH_DIR ${PROJECT_SOURCE_DIR}/deps/altmath)
set(altmath_build_tests OFF)
add_subdirectory(${ALTMATH_DIR})

option(USE_AVX "Use AVX SIMD Instructions." ON)
if (USE_AVX)
    add_compile_definitions(USE_AVX_SIMD)
endif()

set(LIBRARY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB_RECURSE LIBRARY_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_subdirectory(src)
add_subdirectory(demo)
