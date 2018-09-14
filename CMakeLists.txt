cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME fluid_sim)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")

file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS src/*.h)
file(GLOB_RECURSE PROJECT_SHADERS src/shaders/*.vert src/shaders/*.frag src/shaders/*.geom)

add_executable(${PROJECT_NAME} ${PROJECT_HEADERS} ${PROJECT_SOURCES} ${PROJECT_SHADERS})

find_package(OpenGL REQUIRED)
find_package(GLFW3 REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL ${GLFW3_LIBRARY})

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
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/examples/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/examples/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${GLFW3_INCLUDE_DIR})
target_include_directories(imgui PUBLIC ${IMGUI_DIR})
target_include_directories(imgui PUBLIC ${IMGUI_DIR}/examples)
target_link_libraries(imgui PRIVATE glad)
target_link_libraries(imgui PRIVATE ${GLFW3_LIBRARY})
target_link_libraries(${PROJECT_NAME} PRIVATE imgui)

# STB
set(STB_DIR deps/stb)
target_include_directories(${PROJECT_NAME} PRIVATE ${STB_DIR})

# Eigen
set(EIGEN_DIR deps/eigen-3.3.5)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN_DIR})

