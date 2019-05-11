cmake_minimum_required(VERSION 2.8.3)
project(iris_rtk_ros)

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rosbag
    nav_msgs
    iris_rtk
    sensor_msgs
    tf_conversions
    geometry_msgs
    message_filters
    visualization_msgs
)

find_package(octomap REQUIRED)

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES iris-rtk
    #DEPENDS
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(src)

#############
## Install ##
#############

## Mark cpp header files for installation
#install(DIRECTORY include/rtk/ros/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)


