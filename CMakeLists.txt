cmake_minimum_required(VERSION 3.11)
set(PROJECT_NAME fluid_sim)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -Wl,--stack,0x100000000")

file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS src/*.h)
file(GLOB_RECURSE PROJECT_SHADERS src/shaders/*.vert src/shaders/*.frag src/shaders/*.geom)

add_executable(${PROJECT_NAME} ${PROJECT_HEADERS} ${PROJECT_SOURCES} ${PROJECT_SHADERS})

# SDL2
if (MINGW)
    set(SDL2_PATH "C:\\Libraries\\SDL2-2.0.8-MinGW\\x86_64-w64-mingw32")
endif(MINGW)
find_package(SDL2 REQUIRED)
target_include_directories(${PROJECT_NAME} PRIVATE ${SDL2_INCLUDE_DIR})

# GLAD
set(GLAD_DIR deps/glad)
add_library(glad ${GLAD_DIR}/src/glad.c)
target_include_directories(glad PUBLIC ${GLAD_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE glad ${CMAKE_DL_LIBS})

# altlib
set(ALTLIB_DIR deps/altlib)
add_subdirectory(${ALTLIB_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE altlib)

# IMGUI
set(IMGUI_DIR deps/imgui-1.65)
add_library(imgui
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/examples/imgui_impl_sdl.cpp
        ${IMGUI_DIR}/examples/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${SDL2_INCLUDE_DIR})
target_include_directories(imgui PUBLIC ${IMGUI_DIR})
target_include_directories(imgui PUBLIC ${IMGUI_DIR}/examples)
target_link_libraries(imgui PRIVATE glad)
target_link_libraries(imgui PRIVATE SDL2)
target_link_libraries(${PROJECT_NAME} PRIVATE imgui)

# STB
set(STB_DIR deps/stb)
target_include_directories(${PROJECT_NAME} PRIVATE ${STB_DIR})

# Eigen
set(EIGEN_DIR deps/eigen-3.3.5)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN_DIR})

# Handmade Math
set(HMM_DIR deps/HandmadeMath)
target_include_directories(${PROJECT_NAME} PRIVATE ${HMM_DIR})