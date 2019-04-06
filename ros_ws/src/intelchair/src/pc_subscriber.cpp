#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "CommHandler.h"
#include "intelchair/ChairConnection.h"
#include "intelchair/ChairVelocity.h"
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
char connectOption = 0x00;
int buttonPressed = 0x00;

void resetButtons(){
	connectOption = 0x00;
	buttonPressed = 0x00;
}

bool connectionServiceCallback(intelchair::ChairConnection::Request &req, intelchair::ChairConnection::Response &res){
    ROS_INFO("REQUEST CONNECT: %s", req.connection.c_str());
    res.response = true;
    // este response tem de ser colocado a true, apenas quando a propria cadeira responder
    connectOption = 0x01;
    return true;
}

bool velocityServiceCallback(intelchair::ChairVelocity::Request &req, intelchair::ChairVelocity::Response &res){
    ROS_INFO("VELOCITY CHANGE: %s", req.velocity.c_str());
    
    res.response = true;
    // este response tem de ser colocado a true, apenas quando a propria cadeira responder
    std::string str = req.velocity.c_str();
	char info = str.at(0);
    int currentVelocity = chair.velocity;
	if(info == '+'){
		buttonPressed = 0x04;
	}else{
		buttonPressed = 0x02;
	}
    
    return true;
}




void joystickTopicCallback(const geometry_msgs::Point::ConstPtr& msg){

    joystick.x = ((int)msg->x);
    joystick.y = ((int)msg->y); 
    ROS_INFO("Joystick info: (%f, %f)", msg->x, msg->y);


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

	commHandler.sendFrame(joystick, buttonPressed, connectOption);
	// !! THIS CALL IS NOT FINAL. SERVICES NEED TO BE ADDED !! //
	resetButtons();
}

void receiveFrame(const ros::TimerEvent& event){
    // Call commSerial RX here
    //ROS_INFO("RECEIVING FRAME!!!");
	chair = commHandler.receiveFrame();
    intelchair::ChairMsg msg;
    msg.velocity = chair.velocity;
    msg.battery = chair.battery;
    msg.connected = chair.connected;
    pc_publisher.publish(msg);
    // ros::spinOnce();

}

int main(int argc, char **argv){
    ros::init(argc, argv, "pcsubscriber");
    
    ros::NodeHandle n;

    pc_publisher = n.advertise<intelchair::ChairMsg>("/chair_info", 1000);


    ros::Timer response_timer = n.createTimer(ros::Duration(RESPONSE_DELAY), receiveFrame);
    ros::Timer send_timer = n.createTimer(ros::Duration(SEND_DELAY), sendFrame);
    //ros::Timer debug_timer = n.createTimer(ros::Duration(2), sendFrame);

    ros::ServiceServer connect_service = n.advertiseService("/connection_service", connectionServiceCallback);
    ros::ServiceServer velocity_service = n.advertiseService("/velocity_service", velocityServiceCallback);

    ROS_INFO("Subscribing to joystick topic... ");
    ros::Subscriber joystick_sub = n.subscribe("/joystick", 1000, joystickTopicCallback);

	ros::spin();
}