# Developer

Looking at the the [architecture](../architecture/system.md) page we can clearly split our main system in three distinct parts. In this section we are creating some directives on how to deploy each of these individual system parts, and even though it is not an in depth, detailed guide, it helps taking the right steps in to re-creating this project from scratch.

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


------------------------
temporary


We are using [ROS](http://www.ros.org), which is a collection of software libraries and frameworks for robot development. This is the core part of the chair system. This allows us to have different nodes communicating between each other, exchanging information such as sensor data, robot control data or velocity commands for the robot wheels.
Besides the different sensor nodes and drivers (soon to be shown how to deploy) that serve the purpose of creating an abstraction to the sensor value reading as well as publishing that same information in the ROS environment, we implemented some of our own.

## Manual Control
First, and probably the most important node of the system is the base controller node. This is the closest node to the gateway microcontroller of the wheelchair. It is responsible, not only, of reading the latest velocity comands sent by the user, or the navigation stack (mentioned down below), sending them to said gateway and making the chair move, but also of publishing all the control information that the chair responds with.
Since it exchanges information using a serial communication(with microcontroller's UART) we have also created an extra abstraction level with the "CommHandler.cpp" class.
## Autonomous Navigation
The previously mentioned ROS provides an autonomous [navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup) base setup called navigation stack, that simplifies some concepts when talking about this subject.
Another one of the implemented nodes is the "cmdvel_parser.cpp". Because the data structure that the microcontroller is expecting in the low level is different from the navigation stack's output commands for the wheelchair's movement, we created a simple node to parse the return values of the navigation stack and match the values that the microcontroller is expecting, in this specific case, joystick x and y values ranging from -100 and 100.


---
# Server

The server is the centerpiece of our system, as ensures all the communications between the web application, the wheelchair, the dashboard and the database. It is what connects everything in order to have a functional product. We decided to use [Flask](http://flask.pocoo.org/docs/1.0/) to build our application and [SQLite](https://www.sqlite.org/index.html) to manage our database. Before you test anything, make sure you install the packages at the Prerequisites section below.

## Prerequisites

* SQLite 3
* Flask with flask-cors

## How to run

- **In a new terminal run:** ./run_server.sh<br>
This is a simple script to bring the server online.

## What does it feature?

You can interact with 4 entities:

- **Users**
- **Chairs**
- **History of usage**
- **Maps**

## Users
### List all users

**Definition**

`GET /users`

### Registering a new user

**Definition**

`POST /users`

**Arguments**

`"firstname":string` -
`"lastname":string` -
`"username":string` -
`"password"':string` -
`"email":string` -
`"age":int` -
`"role":string` -
`"status":string`


If a user with the given username already exists, the server will not add or overwrite.


### Updating an user

**Definition**

`PUT /users/<username>`

**Arguments**

`"firstname":string` -
`"lastname":string` -
`"password"':string` -
`"email":string` -
`"age":int` - 
`"role":string` -
`"status":string`


### Lookup user details

**Definition**

`GET /users/<username>`

### Delete a user

**Definition**

`DELETE /users/<username>`

<br>

# Chairs
### List all chairs

**Definition**

`GET /chairs`

### Registering a new chair

**Definition**

`POST /chairs`

**Arguments**

`"company":string` -
`"model":string` -
`"name":string` -
`"id":string` - 
`"ip":string` -
`"user":string` -
`"status":string` -
`"battery":integer`


If a chair with the given name already exists, the server will not add or overwrite.


### Updating a chair

**Definition**

`PUT /chairs/<id>`

**Arguments**

`"company":string` -
`"model":string` -
`"name":string` -
`"ip":string` -
`"user":string` -
`"status":string` -
`"battery":integer`


### Lookup chair details

**Definition**

`GET /chairs/<id>`


### Delete a chair

**Definition**

`DELETE /chairs/<id>`

<br>

# History
### List history

**Definition**

`GET /history`

### Add history

**Definition**

`POST /history`

**Arguments**

`startTime:string` -
`endTime:string` -
`username:string` -
`chair:string`


### Delete history

**Definition**

`DELETE /history`

<br>

# Maps
### List all maps

**Definition**

`GET /maps`


### Add map

**Definition**

`POST /maps`

**Arguments**

`name:string` -
`pgm_path:string`

### Update map

**Definition**

`PUT /maps/<name>`

**Arguments**

`"name":string` -
`"pgm_path":string`


### Lookup specific map

**Definition**

`GET maps/<name>`


### Delete map

**Definition**

`DELETE /maps/<name>`
