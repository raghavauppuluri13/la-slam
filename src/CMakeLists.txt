find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/local/include/eigen3")
add_executable(main main.cpp vo.cpp visualize.cpp common.cpp vo_dataloader.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${Pangolin_LIBRARIES}) 
