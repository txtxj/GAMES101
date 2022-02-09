cmake_minimum_required (VERSION 2.8.11)
project (pa0)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_executable (pa0 main.cpp)