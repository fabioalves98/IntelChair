# Architecture

![arch](img/architecture.png)


# Server side

A server handling database requests will be listening for user connections. This database will contain login information aswell as other relevant data necessary to create a link between the user and the chair itself. It will also store the various maps of the different locations where the chair operates.

# Wheelchair

The core part of the wheelchair, linking every node in the system, will be a laptop. It communicates with the chair's control unit through a gateway microcontroller and processes information about every sensor in the chair.
The chair holds a camera for object recognition and collision detection, aswell as a Rangefinder sensor and IMU for the room mapping and navigation.

# Web application

The Web application will allow the user to either take control of the wheelchair or choose to let it do the work autonomously. It enables the user to define default locations to the current working map and travel between those, or simply travel to an arbitrary location chosen by him. It also displays some information about the chair's current battery levels and the current maximum speed.

