
#ifndef RTK_SLAM2D_ROS_H_
#define RTK_SLAM2D_ROS_H_

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
#include <geometry_msgs/PoseArray.h>
// Laser message
#include <sensor_msgs/LaserScan.h>
// maps
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <rtk/slam/pf_slam2d.h>
#include <rtk/geom/pose3d.h>

namespace rtk {

class PFSlam2DROS {
public:

    PFSlam2DROS();
    ~PFSlam2DROS();

    void onLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan);
    bool onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

    void fromBag(const std::string bag_file);

private:
    bool OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg);
    bool DistanceMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg);
    bool PatchMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg);

private:

    // == ROS stuff ==
    ros::NodeHandle nh_;  ///< Root ros node handle.
    ros::NodeHandle pnh_; ///< Private ros node handle.

    tf::TransformBroadcaster* tfb_; ///< Position transform broadcaster.
    tf::TransformListener*    tf_;  ///< Gloabal transform listener.

    tf::Transform latest_tf_; ///< The most recent transform.

    tf::MessageFilter<sensor_msgs::LaserScan>*           laser_scan_filter_; ///< Transform and LaserScan message Syncronizer.
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;    ///< Subscriber to the LaserScan message.

    geometry_msgs::PoseArray poses_;

    // publishers
    ros::Publisher pose_pub_;   ///< Publisher of the pose with covariance.
    ros::Publisher map_pub_;
    ros::Publisher dist_pub_;
    ros::Publisher patch_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher path_pub_;

    ros::ServiceServer ss_;

    // == Laser stuff ==
    // allow to handle multiple lasers at once
    std::map<std::string, int> frame_to_laser_; ///< Map with the known lasers.
    std::vector<bool>          lasers_update_;  ///< Vector that signals which laser to update.
    std::vector<rtk::geom::Pose3D> lasers_origin_;  ///< Laser origin transformation
    double max_range_;

    // maps
    nav_msgs::OccupancyGrid ros_occ_;
    nav_msgs::OccupancyGrid ros_cost_;
    nav_msgs::OccupancyGrid ros_patch_;

    // == configuration variables ==
    std::string global_frame_id_;       ///< Global frame id, usualy the map frame.
    std::string odom_frame_id_;         ///< Odometry frame id.
    std::string base_frame_id_;         ///< Robot base frame.

    std::string scan_topic_;   ///< LaserScan message topic.

    ros::Duration transform_tolerance_;   ///< Defines how long map->odom transform is good for.
    ros::WallDuration map_publish_period_;
    ros::WallTime map_publish_last_time_;

    // == Inner state ==
    rtk::slam::PFSlam2D* slam2d_;
    rtk::geom::Pose2D    odom_;
};

} /* rtk */

#endif /* end of include guard: RTK_SLAM2D_ROS_H_ */
