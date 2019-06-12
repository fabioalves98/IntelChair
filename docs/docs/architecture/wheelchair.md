The core part of the wheelchair, linking every node in the system, is a laptop. It communicates with the wheelchair's control unit through a gateway microcontroller and processes information about every sensor in the wheelchair.
The wheelchair holds a camera for object recognition and collision detection, as well as a laser rangefinder and IMU for room mapping and navigation.
In the software department, 





We are using [ROS](http://www.ros.org), which is a collection of software libraries and frameworks for robot development. This is the core part of the chair system. This allows us to have different nodes communicating between each other, exchanging information such as sensor data, robot control data or velocity commands for the robot wheels.
Besides the different sensor nodes and drivers (soon to be shown how to deploy) that serve the purpose of creating an abstraction to the sensor value reading as well as publishing that same information in the ROS environment, we implemented some of our own.

## Manual Control
First, and probably the most important node of the system is the base controller node. This is the closest node to the gateway microcontroller of the wheelchair. It is responsible, not only, of reading the latest velocity comands sent by the user, or the navigation stack (mentioned down below), sending them to said gateway and making the chair move, but also of publishing all the control information that the chair responds with.
Since it exchanges information using a serial communication(with microcontroller's UART) we have also created an extra abstraction level with the "CommHandler.cpp" class.
## Autonomous Navigation
The previously mentioned ROS provides an autonomous [navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup) base setup called navigation stack, that simplifies some concepts when talking about this subject.
Another one of the implemented nodes is the "cmdvel_parser.cpp". Because the data structure that the microcontroller is expecting in the low level is different from the navigation stack's output commands for the wheelchair's movement, we created a simple node to parse the return values of the navigation stack and match the values that the microcontroller is expecting, in this specific case, joystick x and y values ranging from -100 and 100.
