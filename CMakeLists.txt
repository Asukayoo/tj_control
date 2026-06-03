cmake_minimum_required(VERSION 3.15)
project(tj_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(third_part/ruckig_part)
add_subdirectory(mv_control)
add_subdirectory(examples)
