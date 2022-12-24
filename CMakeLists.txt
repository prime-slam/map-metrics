cmake_minimum_required(VERSION 3.18.0)
set(CMAKE_CXX_STANDARD 14)
project(map_metrics LANGUAGES CXX DESCRIPTION "Library for odometry trajectory estimation")

add_subdirectory(external)
add_subdirectory(cpp)