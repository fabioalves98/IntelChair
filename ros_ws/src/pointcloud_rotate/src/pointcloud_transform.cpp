#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 

void callback(const PointCloud::ConstPtr cloud){
  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI/2; // The angle of rotation in radians
  
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);

  tf_pub.publish(transformed_cloud);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pointcloud_transform");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<PointCloud>("/openni/depth_registered/points", 1, callback);

  tf_pub = n.advertise<sensor_msgs::PointCloud2> ("/openni/depth_registered/tf_pointcloud", 1);

  tf_listener = new tf::TransformListener();

  ros::spin();
  return 0;
};