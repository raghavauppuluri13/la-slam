cmake_minimum_required(VERSION 2.8)
project(la-slam)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)
add_subdirectory(src)
add_subdirectory(test)
