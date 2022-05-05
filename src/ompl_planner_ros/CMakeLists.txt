cmake_minimum_required(VERSION 3.0.2)
project(ompl_planner)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED 
COMPONENTS 
  system 
  serialization
)

find_package(OMPL)
if(NOT OMPL_FOUND)
  message(SEND_ERROR "Failed to find OMPL! Install OMPL and try again.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
    std_msgs
    geometry_msgs
    nav_msgs   
)

add_library(${PROJECT_NAME} src/ompl_planner.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node test/ompl_core.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)