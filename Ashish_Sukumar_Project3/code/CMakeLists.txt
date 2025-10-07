cmake_minimum_required(VERSION 3.10)

project(Project3)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(plan plan.cpp)
target_link_libraries(plan ${OMPL_LIBRARIES})
