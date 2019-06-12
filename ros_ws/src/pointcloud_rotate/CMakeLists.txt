cmake_minimum_required(VERSION 2.8.3)

project(pointcloud_rotate)

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
					tf
					PCL)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS}
		    ${PCL_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(tf_pointcloud src/pointcloud_transform.cpp)

target_link_libraries(tf_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
