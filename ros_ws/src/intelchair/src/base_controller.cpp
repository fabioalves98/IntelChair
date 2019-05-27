#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "CommHandler.h"
#include "intelchair/ChairMsg.h"

#include <sstream>

// Valores iguais ao codigo em "MainWindow.cpp"
#define RESPONSE_DELAY 0.008 
#define SEND_DELAY 0.05
#define DEVICE "/dev/ttyUSB0"

CommHandler commHandler;    
Coordinate joystick;
ChairInfo chair;
ros::Publisher pc_publisher;

char aux[1024 * 4];
int buttonPressed = 0x00;
bool connected = false;

void resetButtons()
{
	buttonPressed = 0x00;
}

void joystickTopicCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    joystick.x = ((int)msg->x);
    joystick.y = ((int)msg->y); 
    // ROS_INFO("Joystick info: (%f, %f)", msg->x, msg->y);
}

void chairInfoCallback(const intelchair::ChairMsg::ConstPtr& msg)
{
    if(msg->velocity > chair.velocity)
    {
        buttonPressed = 0x04;
        ROS_INFO("Velocity up!");
    }
    else if(msg->velocity < chair.velocity)
    {
        ROS_INFO("Velocity Down!");
		buttonPressed = 0x02;
    }
}

void sendFrame(const ros::TimerEvent& event)
{
    connected = commHandler.sendFrame(joystick, buttonPressed);
	resetButtons();
}

void receiveFrame(const ros::TimerEvent& event)
{

    chair = commHandler.receiveFrame();
    intelchair::ChairMsg msg;
    msg.velocity = chair.velocity;
    msg.battery = chair.battery;
    msg.connected = chair.connected;
    pc_publisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcsubscriber");
    
    ros::NodeHandle n;

    pc_publisher = n.advertise<intelchair::ChairMsg>("/chair_info", 1000);

    ros::Timer response_timer = n.createTimer(ros::Duration(RESPONSE_DELAY), receiveFrame);
    ros::Timer send_timer = n.createTimer(ros::Duration(SEND_DELAY), sendFrame);
    //ros::Timer debug_timer = n.createTimer(ros::Duration(2), sendFrame);

    ROS_INFO("Subscribing to joystick topic... ");
    ros::Subscriber joystick_sub = n.subscribe("/joystick", 1000, joystickTopicCallback);
    ros::Subscriber chair_info = n.subscribe("/chair_info_control", 1000, chairInfoCallback);

	ros::spin();
}