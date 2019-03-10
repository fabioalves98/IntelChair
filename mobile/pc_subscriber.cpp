#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

#include <sstream>


/*
    Running this node:
        - Create a ws + new package
        - Edit the CMakeLists file and add this node as an executable
        - rosrun 'package' 'thisnode'
        http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B)

*/

// Valores iguais ao codigo em "MainWindow.cpp"
#define RESPONSE_DELAY 0.008 
#define SEND_DELAY 0.05
#define DEVICE "/dev/ttyUSB0"


struct Coordinate{
    int x;
    int y;
};

char aux[1024 * 4];

void buildMoveFrame(Coordinate joystick){
    int buttonPressed = 0x00;
    int connectOption = 0x00;
    sprintf(aux, "#%c%03d%c%03d%1d%1d", (joystick.x > 0 ? '+' : '-'), abs(joystick.x), (joystick.y > 0 ? '+' : '-'), abs(joystick.y), buttonPressed, connectOption);

}

void moveFrame(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("Joystick info: (%f, %f)", msg->x, msg->y);
    // char aux[1024 * 4];
    // Default trama

    Coordinate joystick;
    joystick.x = msg->x;
    joystick.y = msg->y; 

    buildMoveFrame(joystick);

    
}

void sendFrame(const ros::TimerEvent& event){
    // Call commSerial TX here
    ROS_INFO("AUX VAR: %s", aux);
}

void receiveFrame(const ros::TimerEvent& event){
    // Call commSerial RX here
    ROS_INFO("RECEIVING FRAME!!!");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pcsubscriber");

    ros::NodeHandle n;
    ros::Timer response_timer = n.createTimer(ros::Duration(RESPONSE_DELAY), receiveFrame);
    ros::Timer send_timer = n.createTimer(ros::Duration(SEND_DELAY), sendFrame);


    ROS_INFO("Subscribing to joystick topic... ");
    ros::Subscriber sub = n.subscribe("/joystick", 1000, moveFrame);
    ros::spin();

}