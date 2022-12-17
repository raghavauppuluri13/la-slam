cmake_minimum_required(VERSION 2.8)
project(la-slam)

#set(CMAKE_CXX_FLAGS "-std=c++11")

#add_subdirectory(useEigen)
#add_subdirectory(useGeometry)
#add_subdirectory(visualizeGeometry)
include_directories(include)
add_subdirectory(src)
