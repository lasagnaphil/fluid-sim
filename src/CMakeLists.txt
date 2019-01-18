file(GLOB LIBRARY_INTERNAL_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB LIBRARY_INTERNAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(fluid_sim ${LIBRARY_HEADERS} ${LIBRARY_INTERNAL_HEADERS} ${LIBRARY_INTERNAL_SOURCES})

target_include_directories(fluid_sim
    PUBLIC ${LIBRARY_INCLUDE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

# altlib
target_include_directories(fluid_sim PUBLIC altlib)
target_link_libraries(fluid_sim PUBLIC altlib)

# altmath
target_include_directories(fluid_sim PUBLIC altmath)
target_link_libraries(fluid_sim PUBLIC altmath)

# PThreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(fluid_sim PRIVATE Threads::Threads)

# OpenMP
find_package(OpenMP)
if (OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

