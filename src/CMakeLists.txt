find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")
add_executable(vo vo.cpp visualize.cpp)
target_link_libraries(vo ${OpenCV_LIBS} ${Pangolin_LIBRARIES}) 
