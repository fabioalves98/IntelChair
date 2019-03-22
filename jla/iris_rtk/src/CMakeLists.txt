
add_subdirectory(_vendor/ldc)

set(rtk_SRCS
    print.cpp
    thread_pool.cpp
    # Geometry
    geom/pose2d.cpp
    geom/pose3d.cpp
    geom/circle.cpp
    geom/pinhole.cpp
    geom/line.cpp
    geom/line_segment.cpp
    # I/O
    io/image_io.cpp
    # Lossless data compression
    ldc/lz4_buffer_compressor.cpp
    ldc/pithy_buffer_compressor.cpp
    ldc/zstd_buffer_compressor.cpp
    ldc/quicklz_buffer_compressor.cpp
    # Math
    math/random.cpp
    math/stats.cpp
    # Non-linear Least Squares Optimization
    nlls/dog_leg.cpp
    nlls/gauss_newton.cpp
    nlls/levenberg_marquardt.cpp
    nlls/robust_cost.cpp
    nlls/rprop.cpp
    nlls/solver.cpp
    # Sparse Dense Mapping
    sdm/container.cpp
    sdm/map.cpp
    sdm/dynamic_distance_map.cpp
    sdm/frequency_occupancy_map.cpp
    sdm/probabilistic_occupancy_map.cpp
    sdm/simple_occupancy_map.cpp
    sdm/truncated_signed_distance_map.cpp
    sdm/export.cpp
    # SLAM
    slam/loc2d.cpp
    slam/match_surface_2d.cpp
    slam/match_surface_3d.cpp
    slam/pf_slam2d.cpp
    slam/slam2d.cpp
    # Time
    time/duration.cpp
    time/frequency.cpp
    time/rate.cpp
    time/time.cpp
    time/timer.cpp
)

add_library(iris-rtk ${rtk_SRCS})

target_link_libraries(iris-rtk ${Boost_LIBRARIES} ldc)

####################################################################################################
# Applications
####################################################################################################

add_executable(slam2d apps/slam2d.cpp apps/carmen.cpp)
target_link_libraries(slam2d iris-rtk)

add_executable(pf_slam2d apps/pf_slam2d.cpp apps/carmen.cpp)
target_link_libraries(pf_slam2d iris-rtk)

####################################################################################################
# Install
####################################################################################################

install(TARGETS iris-rtk slam2d pf_slam2d
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
