cmake_minimum_required(VERSION 2.8.3)
project(linemod_lib)

find_package(catkin REQUIRED
  roscpp tf std_msgs sensor_msgs geometry_msgs roslib
  dynamic_reconfigure
  eigen_conversions
  message_generation
  linemod_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES linemod_lib
  CATKIN_DEPENDS
  roscpp tf std_msgs sensor_msgs geometry_msgs roslib linemod_msgs
)

add_definitions(-std=c++11)

add_library(linemod_lib
  src/linemod_lib.cpp
)

target_link_libraries(linemod_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
