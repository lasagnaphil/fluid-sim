file(GLOB DEMO_SOURCES *.cpp)
file(GLOB DEMO_HEADERS *.h)

add_executable(fluid_sim_demo ${DEMO_SOURCES} ${DEMO_HEADERS})

# target_compile_definitions(fluid_sim_demo PUBLIC ALTMATH_USE_SIMD)

target_link_libraries(fluid_sim_demo LINK_PUBLIC fluid_sim)
target_link_libraries(fluid_sim_demo PUBLIC altlib)
target_link_libraries(fluid_sim_demo PUBLIC altmath)

# SDL2
find_package(SDL2 REQUIRED)
target_include_directories(fluid_sim_demo PRIVATE ${SDL2_INCLUDE_DIR})

# GLAD
set(GLAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/glad)
add_library(glad ${GLAD_DIR}/src/glad.c)
target_include_directories(glad PUBLIC ${GLAD_DIR}/include)
target_link_libraries(fluid_sim_demo PRIVATE glad ${CMAKE_DL_LIBS})

# IMGUI
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/imgui-1.65)
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
target_link_libraries(fluid_sim_demo PRIVATE imgui)

# STB
set(STB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/stb)
target_include_directories(fluid_sim_demo PRIVATE ${STB_DIR})

