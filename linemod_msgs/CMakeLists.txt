cmake_minimum_required(VERSION 2.8.3)
project(linemod_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Scored2DBox.msg
  Scored2DBoxArray.msg
  )

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
)
