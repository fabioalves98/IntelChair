#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "intelchair/ChairConnection.h"
#include "intelchair/ChairVelocity.h"
#include "intelchair/ChairMsg.h"

#include <sstream>

ros::Publisher pub;


void parseCmdvel(const geometry_msgs::Twist::ConstPtr& msg){
 
    geometry_msgs::Point j;
    j.x = msg->linear.x * 30;
    
    // Martelado
    if (j.x < 20 && j.x > 0)
    {
        j.x = 20;
    }
    else if (j.x < 0)
    {
        j.x = -20;
    }
    
    j.y = -1 * msg->angular.z * 100;
    ROS_INFO("JOYSTICK(X,Y): (%f, %f)", j.x, j.y);

    pub.publish(j);
    ros::spinOnce(); // important for later callback usage

}

int main(int argc, char **argv){
    ros::init(argc, argv, "cmdvel_parser");
    
    ros::NodeHandle n;

    ROS_INFO("Subscribing to cmd_vel topic... ");
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, parseCmdvel);
    pub = n.advertise<geometry_msgs::Point>("/joystick", 1000);
   
	ros::spin();
}