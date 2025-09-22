cmake_minimum_required(VERSION 3.10)

project(Project2)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

# include RTP.cpp, CollisionChecking.cpp as a library
add_library(MY_LIBRARY CollisionChecking.cpp RTP.cpp)
link_libraries(MY_LIBRARY ${OMPL_LIBRARIES})

add_executable(planRTP PlanningRTP.cpp)
target_link_libraries(planRTP MY_LIBRARY  ${OMPL_LIBRARIES})

add_executable(kinematic_bench
    KinematicChain.cpp
    KinematicChain.h
)
target_include_directories(kinematic_bench PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(kinematic_bench PRIVATE ${OMPL_LIBRARIES})
