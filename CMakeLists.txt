cmake_minimum_required(VERSION 3.11)
project(fluid_sim_project)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(MINGW)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++ -Wl,-Bstatic -lstdc++ -lpthread -Wl,-Bdynamic")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,-Bdynamic,--whole-archive")
endif(MINGW)

set(ALTLIB_DIR ${PROJECT_SOURCE_DIR}/deps/altlib)
add_subdirectory(${ALTLIB_DIR})

set(ALTMATH_DIR ${PROJECT_SOURCE_DIR}/deps/altmath)
set(altmath_build_tests OFF)
add_subdirectory(${ALTMATH_DIR})


set(LIBRARY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB_RECURSE LIBRARY_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_subdirectory(src)
add_subdirectory(demo)
