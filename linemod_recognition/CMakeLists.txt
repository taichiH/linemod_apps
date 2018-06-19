cmake_minimum_required(VERSION 2.8.3)
project(linemod_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  message_generation
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  linemod_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
INCLUDE(FindPkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(linemod_sample src/linemod_sample.cpp)
target_link_libraries(linemod_sample ${catkin_LIBRARIES})

add_executable(linemod_no_depth src/linemod_no_depth.cpp)
target_link_libraries(linemod_no_depth ${catkin_LIBRARIES})

