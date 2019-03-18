#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "CommHandler.h"
#include "intelchair/ChairConnection.h"

#include <sstream>


// Valores iguais ao codigo em "MainWindow.cpp"
#define RESPONSE_DELAY 0.008 
#define SEND_DELAY 0.05
#define DEVICE "/dev/ttyUSB0"


CommHandler commHandler;
Coordinate initial;    
Coordinate joystick;
char aux[1024 * 4];
char connectOption = 0x00;
int buttonPressed = 0x00;

void resetButtons(){
	connectOption = 0x00;
	buttonPressed = 0x00;
}

bool connectionServiceCallback(intelchair::ChairConnection::Request &req, 
                    intelchair::ChairConnection::Response &res)
{
    ROS_INFO("REQUEST CONNECT: %s", req.connection.c_str());
    res.response = true;
    // este response tem de ser colocado a true, apenas quando a propria cadeira responder
    connectOption = 0x01;
    return true;
}

void joystickTopicCallback(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("Joystick info: (%f, %f)", msg->x, msg->y);

    joystick.x = ((int)msg->x) / 3;
    joystick.y = ((int)msg->y) / 3; 

}

void connectionTopicCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Connection info: %s", msg->data.c_str());
	connectOption = 0x01;	
}

void velocityTopicCallBack(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Velocity info: %s", msg->data.c_str());
	std::string str = msg->data.c_str();
	char info = str.at(0);
	if(info == '+'){
		buttonPressed = 0x04;
	}else{
		buttonPressed = 0x02;
	}
}

void sendFrame(const ros::TimerEvent& event){

	commHandler.sendFrame(joystick, connectOption, buttonPressed);
	// !! THIS CALL IS NOT FINAL. SERVICES NEED TO BE ADDED !! //
	resetButtons();
}

void receiveFrame(const ros::TimerEvent& event){
    // Call commSerial RX here
    //ROS_INFO("RECEIVING FRAME!!!");
	commHandler.receiveFrame();

}

int main(int argc, char **argv){
    ros::init(argc, argv, "pcsubscriber");
	


    ros::NodeHandle n;
    ros::Timer response_timer = n.createTimer(ros::Duration(RESPONSE_DELAY), receiveFrame);
    ros::Timer send_timer = n.createTimer(ros::Duration(SEND_DELAY), sendFrame);
    //ros::Timer debug_timer = n.createTimer(ros::Duration(2), sendFrame);

    ros::ServiceServer connect_service = n.advertiseService("/connection_service", connectionServiceCallback);

    ROS_INFO("Subscribing to joystick topic... ");
    ros::Subscriber joystick_sub = n.subscribe("/joystick", 1000, joystickTopicCallback);
    
	// ROS_INFO("Subscribing to connection topic... ");
    // ros::Subscriber connection_sub = n.subscribe("/connection", 1000, connectionTopicCallback);
	
	ROS_INFO("Subscribing to velocity topic... ");
    ros::Subscriber sub = n.subscribe("/max_speed", 1000, velocityTopicCallBack);

	ros::spin();

}