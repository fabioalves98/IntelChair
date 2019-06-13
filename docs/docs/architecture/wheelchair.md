The core part of the wheelchair, linking every node in the system, is a laptop. It communicates with the wheelchair's control unit through a gateway microcontroller and processes information about every sensor in the wheelchair.
The wheelchair holds a camera for object recognition and collision detection, as well as a laser rangefinder and IMU for room mapping and navigation.  
In the software department, we are using [ROS](http://www.ros.org), which is a collection of software libraries for robot development. This allows us to have different nodes communicating with each other, exchanging information such as sensor data, wheelchair control data or velocity commands for the wheelchair's wheels.

## Manual Control

![manual](../img/manual.png)

First, and probably the most important node of the system is the base controller node. This is the closest node to the gateway microcontroller of the wheelchair. It is responsible, not only, for reading the latest angular and linear velocity comands sent by the user / navigation stack, sending them to said gateway and making the chair move, but also for publishing all the control information that the chair responds with.

## Mapping

![manual](../img/mapping.png)

## Navigation

![manual](../img/navigation2.png)
