cmake_minimum_required(VERSION 2.8.3)
project(intelchair)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation tf)

add_service_files(
  FILES
  ChairConnection.srv
  ChairVelocity.srv
)

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

## PC_Subscriber
add_executable(subscriber src/pc_subscriber.cpp src/CommSerial.cpp src/CommHandler.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})

add_executable(cmdparser src/cmdvel_parser.cpp)
target_link_libraries(cmdparser ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})