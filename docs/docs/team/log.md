# Logbook

## Week 1 - February 17th

* In this week, we had the first meeting with the mentors 2 days after the project was assigned to us. We discussed the overall scope and objectives since we had no idea of what it would consist of. 
* We were first introduced with the chair’s structure and functions and those were very basic. It has 2 motors, a battery and a joystick and the mentors said they would give us a high-level way of controlling the motors.
* In the end, the mentors recommended us to start familiarizing with some technologies that we would use in the project such as ROS, OpenCV and maplab. 

## Week 2 - February 24th

* The chair’s internal controller documentation and code for the microcontroller that would communicate with it, were sent to us by the mentors, so we could start developing and testing the barebones structure of our system.
* We had another meeting with the mentors, where we tested the code and discussed more details about the functions we would implement. We presented some ideas, like adding a remote server to the system, making the chair follow the user and having support for voice control. In general, they were well accepted.
* At the architecture level, it was decided the chair would have a 360º Laser Rangefinder, an RGBD camera and an Inertial measurement unit sensor.

## Week 3 - March 3rd

* In this week, alongside the mentors we defined the Minimum Viable Product, that we had to present in the following week. All of it’s details can be found in the deliverables section, under Milestone 1. 
* Having the repository available to us, we started creating the documentation website and testing it at xcoa. We also made the interface mockups, the first draft of the user manual and some slides to present our project.
* At the technical level, at this point, we already had a good knowledge of the ROS core functionality, through making various tutorials and exploring examples. With that knowledge we made a minimal web application with a joystick, that runs on our smartphone and sends positional information to the chair’s computer. Upon receiving that information, it transmits it to the chair’s ECU through the gateway microcontroller.
 
## Week 4 - March 10th

* Having all the documentation ready for Milestone 1, we displayed it on the website and finished some aesthetic details for delivery.
* We added more functionality to the web application such as connection options, velocity control and chair’s information.
* In the weekly meeting with the mentors, we were introduced with the actual models of the sensors so we could research on how to control them. The rangefinder was put in place by the mentors at the back of the chair, and they also provided us with a Kinect camera and the IMU sensor. 
* We also started developing an abstraction for the microcontroller, which will allow someone in the future to better control the chair without knowing most of the communication details. This will also be documented in the developer section.

## Week 5 - March 17th

* With all the sensors available to us we discussed with the mentors the best way they could be positioned in the chair. Although the rangefinder was already in it's final place, it still needed power from the chair's batteries. Although the kinect suffered from the same problem, we could just connect it to a power socket and run some tests. Regarding the IMU we had no problem since it was powered over USB. 
* In the weekly meeting, with the help of investigator Eurico Farinha, we were introduced to the proper methods of map generation and navigation with the chair and where to start in those topics.
* After some research we decided which technologies we will use for the application and server, Vue.js and CherryPy respectively. We also found a good library for voice recognition, released some tests but we are still unsure on it's adoption since it's not supported in some browsers.

## Week 6 - March 24th

* We finally have all the sensors in it's final place. Our mentor's made a lot of adaptations to the chair so that the sensors could be in favourable positions, like the Kinect being the front of the chair with unrestricted field of view and the IMU stable in the center of the chair. After some testing we decided the best orientation for the kinect, since it had to detect objects on the floor and also capture the silhouette of the user in the follow mode. 
* At this state, we have all the necessary raw data from the sensors. The next steps will be to process that data so we can feed the algorithms.
* Since we had to deliver the MVP as a formal document in the following week, the majority of topics discussed in the weekly meeting with the mentors, were in that regard, mainly specifing in detail the conditions of the enviroment in which the chair would operate.
* We also made good steps in the navigation section, such as generating odometry values required by the map generation algorithm, using the data from the IMU.


