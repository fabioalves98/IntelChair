cmake_minimum_required(VERSION 2.8)
find_package( OpenCV REQUIRED )
add_library(freenect_cv SHARED libfreenect_cv.cpp)
set_target_properties (freenect_cv PROPERTIES
  VERSION ${PROJECT_VER}
  SOVERSION ${PROJECT_APIVER})

include_directories (../c_sync)

target_link_libraries (freenect_cv freenect_sync ${OpenCV_LIBS})

install (TARGETS freenect_cv DESTINATION ".")
install (FILES "libfreenect_cv.h" DESTINATION .)

add_executable( yolo yolo.cpp )
target_link_libraries(yolo freenect freenect_sync freenect_cv ${OpenCV_LIBS} )
install (TARGETS yolo
  DESTINATION .)