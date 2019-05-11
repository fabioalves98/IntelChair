cmake_minimum_required(VERSION 3.2)
project(iris_rtk)

set(CMAKE_CXX_STANDARD 11)

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost  REQUIRED COMPONENTS program_options)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES iris-rtk
    DEPENDS EIGEN3 Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

add_subdirectory(src)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/rtk/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)


