# Architecture

The main purpose of this high level architecture is to specify all the individual components that are used to make this system work. The three main ones are described below, but it's worth to say what information is transmitted between each one. The base idea is that the web application will initially start a communication with the server, sending the user's login information. The server will then try to establish communication with the chair's laptop to see if the chair is free or not. If it is, then the server will connect the user app to the chair itself. If this process runs successfully, the webapp has the ability to send(or receive) information directly to the chair's laptop node, which then processes that information and sends it to the chair's ECU through our gateway microcontroller.

![arch](img/architecture.png)


# Server side

A server handling database requests will be listening for user connections. This database will contain login information as well as other relevant data necessary to create a link between the user and the chair itself. It will also store the various maps of the different locations where the chair operates.

# Wheelchair

The core part of the wheelchair, linking every node in the system, will be a laptop. It communicates with the chair's control unit through a gateway microcontroller and processes information about every sensor in the chair.
The chair holds a camera for object recognition and collision detection, as well as a Rangefinder sensor and IMU for the room mapping and navigation.

# Web application

The Web application will allow the user to either take control of the wheelchair or choose to let it do the work autonomously. It enables the user to define default locations to the current working map and travel between those, or simply travel to an arbitrary location chosen by him. It also displays some information about the chair's current battery levels and the current maximum speed.

