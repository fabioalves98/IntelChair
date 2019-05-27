cmake_minimum_required(VERSION 2.8.3)
project(intelchair)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation tf)

add_message_files(
  FILES
  ChairMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Base_Controller
add_executable(base_controller src/base_controller.cpp src/CommSerial.cpp src/CommHandler.cpp)
target_link_libraries(base_controller ${catkin_LIBRARIES})

add_executable(cmdvel_parser src/cmdvel_parser.cpp)
target_link_libraries(cmdvel_parser ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})