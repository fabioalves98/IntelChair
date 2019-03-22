
#pragma once

// ROS includes
#include <ros/ros.h>

// Transform include
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tf.h>

#include <message_filters/subscriber.h>

// Pose publishing
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Laser message
#include <sensor_msgs/LaserScan.h>
// maps
#include <nav_msgs/OccupancyGrid.h>

#include <rtk/geom/pose3d.h>
#include <rtk/slam/loc2d.h>

#include <rtk/sdm/simple_occupancy_map.h>
#include <rtk/sdm/dynamic_distance_map.h>

namespace rtk {

class Loc2DROS {
public:

    Loc2DROS();
    ~Loc2DROS();

    void onInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial_pose);
    void onLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan);

    //bool onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

private:

    void InitLoc2DFromOccupancyGridMsg(const nav_msgs::OccupancyGrid& msg);

    bool initLaser(const sensor_msgs::LaserScanConstPtr& laser_scan);

private:

    // == ROS stuff ==
    ros::NodeHandle nh_;  ///< Root ros node handle.
    ros::NodeHandle pnh_; ///< Private ros node handle.

    tf::TransformBroadcaster* tfb_; ///< Position transform broadcaster.
    tf::TransformListener*    tf_;  ///< Gloabal transform listener.

    tf::Transform latest_tf_; ///< The most recent transform.
    ros::Duration transform_tolerance_;   ///< Defines how long map->odom transform is good for.

    tf::MessageFilter<sensor_msgs::LaserScan>*           laser_scan_filter_; ///< Transform and LaserScan message Syncronizer.
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;    ///< Subscriber to the LaserScan message.

    // Publishers
    ros::Publisher pose_pub_; ///< Publishers of the pose with covariance

    // Subscribers
    ros::Subscriber pose_sub_;   ///< Subscriber of the initial pose (with covariance)

    //ros::ServiceServer ss_;

    // == Laser stuff ==
    // Handle multiple lasers at once
    std::map<std::string, int> frame_to_laser_; ///< Map with the known lasers.
    std::vector<bool>          laser_is_reversed_;;  ///< Vector that signals if the laser is reversed
    std::vector<rtk::geom::Pose3D> lasers_origin_;  ///< Laser origin transformation

    // == configuration variables ==
    std::string global_frame_id_;       ///< Global frame id, usualy the map frame.
    std::string odom_frame_id_;         ///< Odometry frame id.
    std::string base_frame_id_;         ///< Robot base frame.

    std::string scan_topic_;   ///< LaserScan message topic.

    // == Inner state ==
    rtk::slam::Loc2D  loc2d_;
    rtk::geom::Pose2D  odom_;
};

} /* rtk */

