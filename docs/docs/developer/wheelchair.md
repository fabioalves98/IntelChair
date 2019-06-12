# Wheelchair

The wheelchair setup itself will probably take the longest of all three parts of the system. It requires having some pre-requisites fulfilled before continuing with the deployment. 
In this case, we are assuming the following requirements are done:

 * There is a laptop with battery charge capacity, one ethernet port and atleast three usb ports ready to connect to the chair's sensors   
* ROS-melodic is [instaled](http://wiki.ros.org/melodic/Installation/Ubuntu).
 * The code repository for the project is cloned and ready.

After all these steps are completed we can begin to start the chair system.
First, the ROS workspace must be compiled, and some dependencies are required for that, navigate to the folder "ros_ws" in the repository and execute:
```
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

After the workspace compiled the wheelchair system is ready to run. In that sense, some ros launch files are provided to create some abstraction on the current existing nodes.

## Basic wheelchair functionality

```
roslaunch intelchair wheelchair.launch
```

This launch runs the basic nodes necessary to run the basic system functionality. In this case, rosbridge creating a connection point for an outside node to connect (the smartphone application in our case), all the basic sensor drivers and publihers such as the laser rangefinder node which publishes a ```/scan``` topic as well as the base wheelchair controller that exchanges information directly with the wheelchair's gateway microcontroller and some static transforms necessary for the later used navigation stack.

It's also worth mentioning that a range finder to odometry node is launched to generate the wheelchair's ```/odometry``` topic and a laser filter is applied to the laser, limiting the laser's field of view and publishing the new filtered laser information to the ```/scan_filtered``` topic.

Finally, a server connection node is also launched creating the first connection between the chair itself and the already running server. Since what this node does essentially is take the wheelchair's laptop ip and send it to the server for it to store, we have to specify the laptop's interface and the correct server ip before running the node.


## Mapping

```
rosrun iris_rtk_ros pf_slam2d_ros scan:=/scan_filtered
```

This node will take care of the map generation and since we previously filtered the laser's output we have to feed the new scan topic in to the mapping algorithm. After initiated, we can use rviz to visualize the current map status with:

```
rosrun rviz rviz
```

and end the current mapping process by stopping the pf_slam2d node and saving the map with:

```
rosrun map_server map_saver -f <filename>
```


## Navigation

```
roslaunch intelchair navigation.launch
```

The navigation launch runs not only the ros navigation stack but also the localization algorithm and the velocity command parser. This node subscribes to the ```/cmd_vel``` topic outputed by the navigation stack when a trajectory to a specified goal is calculated. This information is then parsed and redirected to the base controller node to send to the wheelchair's controller.