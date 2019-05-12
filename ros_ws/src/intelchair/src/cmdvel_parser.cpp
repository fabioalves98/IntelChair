#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "intelchair/ChairConnection.h"
#include "intelchair/ChairVelocity.h"
#include "intelchair/ChairMsg.h"

#include <sstream>

ros::Publisher pub;

float invert_nx(float nx){

    float rx;
    rx = 2 * (2 * (nx - 0.1) + 0.1);
    if(rx/2 > 0.1) return rx;

    rx = 2 * (2 * (nx + 0.1) - 0.1);
    if(rx/2 < -0.1) return rx;

    else return nx * 2;
}


void parseCmdvel(const geometry_msgs::Twist::ConstPtr& msg){
    float v = msg->linear.x;
    float w = msg->angular.z;

    // cmd_vel to wheel velocity
    float vel_right = ((2 * v) + w) / 2; 
    float vel_left =  ((2*v) - w) / 2;

    ROS_INFO("VEL RIGHT : VEL LEFT: (%f, %f)", vel_right, vel_left);


    // Wheel velocity to joystick x and y
    // other options are available.

    /*
    function map_joystickC(x, y){
        x = x / 100;
        y = y / 100;
        var motor_vel = {R: 0, L: 0};
        function nx(x){
            if(x/2 > 0.1) return 0.1+((x/2) - 0.1)/2;
            else if(x/2 < -0.1) return -0.1+((x/2) + 0.1)/2;
            else return x/2;
        }

        motor_vel.R = y + nx(x);
        motor_vel.L = y - nx(x);
        return motor_vel;
    }
    */
    // solving equation system 
    float nx = (vel_right - vel_left) / 2;
    ROS_INFO("NX: %f", nx);
    float y = -nx + vel_right;
    float x = invert_nx(nx);
    ROS_INFO("JOYSTICK(X,Y): (%f, %f)", x, y);

 
    geometry_msgs::Point j;
    j.x = x;
    j.y = y;


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