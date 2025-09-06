cmake_minimum_required(VERSION 3.10)

project(Project1)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_library(MY_LIBRARY DiskSampler.cpp)
link_libraries(MY_LIBRARY ${OMPL_LIBRARIES})

# Make the project executable
add_executable(diskPlanning DiskPlanning.cpp)
target_link_libraries(diskPlanning MY_LIBRARY)
