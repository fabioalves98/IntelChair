cmake_minimum_required(VERSION 2.8.3)
project(intelchair)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)

add_service_files(
  FILES
  ChairConnection.srv
  ChairVelocity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## PC_Subscriber
add_executable(subscriber src/pc_subscriber.cpp src/CommSerial.cpp src/CommHandler.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})