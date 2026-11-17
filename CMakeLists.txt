cmake_minimum_required(VERSION 3.16)
project(GP_ACO VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(Threads REQUIRED)
add_executable(gpaco
    src/main.cpp src/core/common.cpp src/core/random.cpp src/problem/simulator.cpp
    src/gp/tree_nodes.cpp src/gp/tree_operations.cpp src/gp/model.cpp
    src/gp/training.cpp src/problem/local_search.cpp src/aco/search.cpp)
target_include_directories(gpaco PRIVATE src third_party)
target_link_libraries(gpaco PRIVATE Threads::Threads)
if(MSVC)
    target_compile_options(gpaco PRIVATE /utf-8 /EHsc /bigobj)
    target_compile_definitions(gpaco PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
endif()
