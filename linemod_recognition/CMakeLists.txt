cmake_minimum_required(VERSION 2.8.3)
project(linemod_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  linemod_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  linemod_msgs
)

add_executable(linemod src/linemod.cpp)
target_link_libraries(linemod ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(linemod_2d src/linemod_2d.cpp)
# target_link_libraries(linemod_2d ${catkin_LIBRARIES})

# add_executable(linemod_no_depth src/linemod_no_depth.cpp)
# target_link_libraries(linemod_no_depth ${catkin_LIBRARIES})

# add_executable(online_template_creator src/online_template_creator)
# target_link_libraries(online_template_creator ${catkin_LIBRARIES})

# add_executable(hough src/hough.cpp)
# target_link_libraries(hough ${catkin_LIBRARIES})

# add_executable(gazebo_line2d src/gazebo_line2d.cpp)
# target_link_libraries(gazebo_line2d ${catkin_LIBRARIES})
