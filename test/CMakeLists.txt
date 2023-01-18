find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/local/include/eigen3")

file(GLOB_RECURSE SRC_FILES ../src/[!main]**.cpp)
file(GLOB_RECURSE TEST_FILES tests/**.cpp)
file(GLOB_RECURSE FIXTURE_FILES fixtures/**.cpp)
add_executable(run_tests main.cpp simpletest.cpp ${TEST_FILES} ${FIXTURE_FILES} ${SRC_FILES})
target_link_libraries(run_tests ${OpenCV_LIBS} ${Pangolin_LIBRARIES}) 
