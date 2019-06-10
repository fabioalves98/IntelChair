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
* Since we have to deliver the MVP as a formal document in the nest week, the majority of topics discussed in the weekly meeting with the mentors, were in that regard, mainly specifying in detail the conditions of the environment in which the chair would operate.
* We also made good steps in the navigation section, such as generating odometry values required by the map generation algorithm, using the data from the IMU.

## Week 7 - March 31st

* As our system was getting bigger and more complex, we started having problems when connecting all wheelchair's modules at the same time along with the mobile application. We dedicated half of the week to solve those problems, to prevent having them in the future, when the system will be even bigger. 
* In the weekly meeting, since we will present the project with a demo next week, we discussed the best method of showing our current progress. After the meeting we recorded some videos and started making the slides we are going to present.
* We also changed the framework for the mobile application. We were going to use Vue.js but after some research and testing, we decided it was not the best option. We then choose the Ionic framework and after applying all the functionality we had made so far in pure HTML and JS to Ionic, we decided to stick with it.

## Week 8 - April 7th

* We presented the M2 Demo this week, and for that reason we spent the first half of the week perfecting the slides, and sanding some edges for the demo. 
* Unfortunately we were not able the schedule a meeting with the mentors to get some feedback on the presentation, but since we had layed out the next steps for the project we kept working on it.
* We are having some problems on generating odometry values using only the IMU. The main problem so far is getting an accurate value for orientation and so far we have been using the magnetometers of the IMU to do so. We had to drop that approach because, as the mentors told us, there are electric wiring beneath the Cambada field that generate electromagnetic fields way bigger than earth's and for that reason, our values we're incorrect and inconsistent.

## Week 9, 10 and 11 - April 14th to 28th

* In these weeks we were also not able the schedule a meeting the the mentors. Because of Easter and the Academic Week, most of us were not available and unfortunately little progress was made.
* We started developing a management dashboard for the server, in which, a system administrator can control wheelchair's access, properties and so much more.
* Because the problems we were having with the IMU we started exploring alternative ways for generating the odometry. Some alternatives suggested by the mentors were using the values sent to the wheelchair's controller or using the sequential laser scans. After some trial and error, the best values we were getting out of the three methods, were generating the odometry from the laser scans. Despite this development we were still not able to generate an usable map for navigation.

## Week 12 - May 5th

* Since we were going to present the M4 Demo this week, we spent the first half making sure the new features were functional and bug free, and made some slides around them.
* At the weekly meeting we received some feedback on the previous demos and discussed what is left to do and how it will be accomplished. 
* With the help of the mentors we found that the problem we were having with odometry generation, came from the fact that the back of the chair was being captured by the laser scans. The solution was to filter the scans and this way we could now generate an accurate map of the room. We still have some problems with ROS parameterization but we can now start studying the navigation algorithms since we now have all the components needed.
* We also made changes to the application and dashboard to accommodate the mapping functions.

## Week 13 - May 12th

* With the map generated and all the other components up and running, we connected everything to the navigation algorithm and were able to achieve our main goal. We can make the wheelchair go from point A to point B by simply giving it a coordinate goal in the map. Despite this big development, we still have a lot of parameterization to perfect and testing to perform.
* At the weekly meeting, we discussed the developments of the project and, since there is not much time left, and the main features are complete, we decided that we will try to implement the extra features we mentioned that were worth doing but did not include in the MVP.
* In this sense, we add some voice commands to the mobile application, we started testing a human recognition module for the kinect and, also using the kinect, started planing how we could include it in the navigation algorithm.

## Week 14 - May 19th

* Since we achieved our goal of navigation in the previous week, this week we gave more focus to the development of the server and mobile application. We concluded our API structure and covered some security flaws. We also developed 2 new sections in the mobile applciation to give mapping and navigation control to the user. 
* At the weekly meeting, we discussed with the mentors logistics for M5 presentation and overall satus of the project.
* For M5 presentation, which will take place next week, we are planning to do a flyer to quickly showcase our project, videos for better visualization of our system and some diagrams to better explain the more technical details.

## Week 15 - May 26th

* At the start of the week, preceding the presentation we took the wheelchair to DETI to do some testing and record some videos. We mapped the DETI hall and one class room, and navigation in those environments was sucessfull. We also recorded videos of all the tests we made. 
* After the presentation and for the rest of the week, we kept ajusting parameters in the navigation module and created some launch files to make the setup process of the wheelchair more straightforward. All of these technical details can be found in the developer page.

## Week 16, 17 - Jun 2nd to 9th

* In these weeks, we mostly developed documentation for our project (manuals, developer page, architecture details). Now that we had some feedback from our mentors from the previous presentation, we adjusted some details accordingly. 
* We made all the necessary preparations for students@DETI presentation.
* Overall, we are happy with our work and the results we achieved.