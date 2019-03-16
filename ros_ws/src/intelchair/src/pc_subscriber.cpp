#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "CommHandler.h"

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


// struct Coordinate{
//     int x;
//     int y;
// };

CommHandler commHandler;
Coordinate initial;    
Coordinate joystick;
char aux[1024 * 4];
// CommSerial comm;
char connectOption = 0x00;
int buttonPressed = 0x00;

// void buildMoveFrame(Coordinate joystick){
//     sprintf(aux, "#%c%03d%c%03d%1d%1d", (joystick.x > 0 ? '-' : '+'), abs(joystick.x), (joystick.y > 0 ? '+' : '-'), abs(joystick.y), buttonPressed, connectOption);
// }

void resetButtons(){
	connectOption = 0x00;
	buttonPressed = 0x00;
}

void joystickTopicCallback(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO("Joystick info: (%f, %f)", msg->x, msg->y);
    // char aux[1024 * 4];
    // Default trama

    joystick.x = ((int)msg->x) / 3;
    joystick.y = ((int)msg->y) / 3; 

    // buildMoveFrame(joystick);
	// commHandler.buildFrame(joystick, connectOption, buttonPressed);
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
    // Call commSerial TX here
    // ROS_INFO("AUX VAR: %s", aux);
    // if(comm.serialTx(aux, strlen(aux)) != 0)
    //   fprintf(stderr, "ERRO ENVIO %s %d\n", aux, (int)strlen(aux));
    // if (connectOption == 0x01)
    // {
    //     connectOption = 0x00;
    //     buildMoveFrame(initial);
    // }

	commHandler.sendFrame(joystick, connectOption, buttonPressed);
	// !! THIS CALL IS NOT FINAL. SERVICES NEED TO BE ADDED !! //
	resetButtons();
}

void receiveFrame(const ros::TimerEvent& event){
    // Call commSerial RX here
    ROS_INFO("RECEIVING FRAME!!!");
	commHandler.receiveFrame();

//     char ch;
//     char response[1024*16] = {0};
//     int count = 0;

//     int fd = comm.fileDescriptor();
//     fd_set read_mask;
//     struct timeval time_out;

//     FD_ZERO(&read_mask);
//     FD_SET(fd, &read_mask);
//     time_out.tv_sec = 0;  
//     time_out.tv_usec = 0;
//     select(fd+1, &read_mask, NULL, NULL, &time_out);

//     if (connectOption == 0)
//       return;

//     while(FD_ISSET(fd, &read_mask) || count < 14){
//       comm.serialRxByte(&ch);
//       response[count] = ch;

//       FD_ZERO(&read_mask);
//       FD_SET(fd, &read_mask);
//       time_out.tv_sec = 0;
//       time_out.tv_usec = 0;
//       select(fd+1, &read_mask, NULL, NULL, &time_out);

//       count++;

//       if (((response[0] != 'F') || (response[1] != 'E')) && (count == 2)){
//          count = 0;
//          continue;
//       }

//       if (((response[2] != '5') || (response[3] != '4')) && (count == 4)){
//          count = 0;
//          continue;
//       }
//     }

//    // Clean buffer in case there's still more information on it to be read
//    while(FD_ISSET(fd, &read_mask)){
//       comm.serialRxByte(&ch);

//       FD_ZERO(&read_mask);
//       FD_SET(fd, &read_mask);
//       time_out.tv_sec = 0;
//       time_out.tv_usec = 0;
//       select(fd+1, &read_mask, NULL, NULL, &time_out);
//    }

   /*
   for (count = 0; count < 14 ; count++)
    {
         fprintf(stderr, "%c",response[count]);
         if ((count + 1) % 2 == 0)
         {
            fprintf(stderr, " ");
         }
      }
      fprintf(stderr, "\n");
    */
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pcsubscriber");
    // comm.openSerial(DEVICE, 115200, 8, PARITY_NONE, 1);
	

    // initial.x = 0;
    // initial.y = 0;
    // buildMoveFrame(initial);

    ros::NodeHandle n;
    //ros::Timer response_timer = n.createTimer(ros::Duration(RESPONSE_DELAY), receiveFrame);
    //ros::Timer send_timer = n.createTimer(ros::Duration(SEND_DELAY), sendFrame);
    ros::Timer debug_timer = n.createTimer(ros::Duration(2), sendFrame);


    ROS_INFO("Subscribing to joystick topic... ");
    ros::Subscriber joystick_sub = n.subscribe("/joystick", 1000, joystickTopicCallback);
    
	ROS_INFO("Subscribing to connection topic... ");
    ros::Subscriber connection_sub = n.subscribe("/connection", 1000, connectionTopicCallback);
	
	ROS_INFO("Subscribing to velocity topic... ");
    ros::Subscriber sub = n.subscribe("/max_speed", 1000, velocityTopicCallBack);

	ros::spin();

}