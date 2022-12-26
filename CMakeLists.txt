cmake_minimum_required(VERSION 3.18.0)
set(CMAKE_CXX_STANDARD 14)
project(map_metrics LANGUAGES CXX DESCRIPTION "Library for odometry trajectory estimation")

option(BUILD_EXAMPLES "Build small usage examples" ON)

add_subdirectory(external)
add_subdirectory(cpp)
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()