#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;
    tf::Quaternion q;

    q.setRPY(0,0,M_PI);

    while(n.ok())
    {
        broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(q, tf::Vector3(-0.07, 0, 0.28)),
            ros::Time::now(), "base_link", "base_laser"
        ));
        r.sleep();
    }
}