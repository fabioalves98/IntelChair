
# Quick notes on running chair mapping

Note: to run a bag make sure to run: $rosparam set use_sim_time true
before anything else
# Using a bag:
1. Run the static transformer:
    rosrun robot_setup_tf tf_broadcaster
2. Initialize the rf2o node:
    roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
3. Start rviz:
    rosrun rviz rviz
4. Start gmapping:
    rosrun gmapping slam_gmapping scan:=scan
5. Start the bag:
    rosbag play --clock bagname

