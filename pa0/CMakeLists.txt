#CMake最低版本号要求
cmake_minimum_required(VERSION 3.12)
#项目信息
project(pa0)
#使用C++ 11标准
set(CMAKE_CXX_STANDARD 11)
#指定生成目标
add_executable(pa0 main.cpp)
