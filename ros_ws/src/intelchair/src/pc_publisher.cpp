#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


/*
    Running this node:
        - Create a ws + new package
        - Edit the CMakeLists file and add this node as an executable
        - rosrun 'package' 'thisnode'
        http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B)
*/


int main(int argc, char **argv){
    ros::init(argc, argv, "pcpublisher");

    ros::NodeHandle n;

    ros::Publisher pc_publisher = n.advertise<std_msgs::String>("/joystick", 1000);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Publishing to joystick test";
        msg.data = ss.str();

        pc_publisher.publish(msg);
        ros::spinOnce(); // important for later callback usage

        loop_rate.sleep();
        ROS_INFO("Publisher call debug");
    }

}