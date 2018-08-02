cmake_minimum_required(VERSION 2.8.3)
project(linemod_3dpose_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
INCLUDE(FindPkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES linemod_3dpose_estimator
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(viewpoint_planner src/viewpoint_planner.cpp)
target_link_libraries(viewpoint_planner ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})

add_executable(linemod src/linemod.cpp)
target_link_libraries(linemod ${catkin_LIBRARIES})
