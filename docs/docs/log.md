# Logbook

## Week 1 - 17 de Fevereiro

* In this week, we had the first meeting with the mentors 2 days after the project was assigned to us. We discussed the overall scope and objectives since we had no idea of what it would consist of. 
* We were first introduced with the chair’s structure and functions and those were very basic. It has 2 motors, a battery and a joystick and the mentors said they would give us a high-level way of controlling the motors.
* In the end, the mentors recommended us to start familiarizing with some technologies that we would use in the project such as ROS, OpenCV and maplab. 

## Week 2 - 24 de Fevereiro

* The chair’s internal controller documentation and code for the microcontroller that would communicate with it, were sent to us by the mentors, so we could start developing and testing the barebones structure of our system.
* We had another meeting with the mentors, where we tested the code and discussed more details about the functions we would implement. We presented some ideas, like adding a remote server to the system, making the chair follow the user and having support for voice control. In general, they were well accepted.
* At the architecture level, it was decided the chair would have a 360º Laser Rangefinder, an RGBD camera and an Inertial measurement unit sensor.

## Week 3 - 3 de Março 

* In this week, alongside the mentors we defined the Minimum Viable Product, that we had to present in the following week. All of it’s details can be found in the deliverables section, under Milestone 1. 
* Having the repository available to us, we started creating the documentation website and testing it at xcoa. We also made the interface mockups, the first draft of the user manual and some slides to present our project.
* At the technical level, at this point, we already had a good knowledge of the ROS core functionality, through making various tutorials and exploring examples. With that knowledge we made a minimal web application with a joystick, that runs on our smartphone and sends positional information to the chair’s computer. Upon receiving that information, it transmits it to the chair’s ECU through the gateway microcontroller.
 
## Week 4 - 10 de Março

* Having all the documentation ready for Milestone 1, we displayed it on the website and finished some aesthetic details for delivery.
* We added more functionality to the web application such as connection options, velocity control and chair’s information.
* In the weekly meeting with the mentors, we were introduced with the actual models of the sensors so we could research on how to control them. The rangefinder was put in place by the mentors at the back of the chair, and they also provided us with a Kinect camera and the IMU sensor. 
* We also started developing an abstraction for the microcontroller, which will allow someone in the future to better control the chair without knowing most of the communication details. This will also be documented in the developer section.
