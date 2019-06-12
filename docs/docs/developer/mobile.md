#Mobile Application
The mobile application is built using HTML, CSS, JavaScript and Ionic. It allows the user to interact with the system by providing features such as manual control and navigation. The app interact  with ROS (wheelchair) through [Roslibjs](http://wiki.ros.org/roslibjs), uses WebSockets to connects with the chair via rosbridge, which provides publishing, subscribing and other essential ROS functionality.

Beyond the front-end in terms of communication with the chair we use functions that work as a set of wrapper methods, that simplify the interation with the ROS system. The following function is an example of that wrapper method, receive the topic's name, a type and the data and publish the information in the right topic:

```
function publish_info(topic, msg_type, data){
	var publisher = new ROSLIB.Topic({
		ros: ros,
		name: topic,
		messageType: msg_type

	});

	publisher.publish(data);
}
```

In addition to the roslibjs, we used the [nipplejs](https://github.com/yoannmoinet/nipplejs) library to create the joystick that allows the manual control of the chair. Access to information about the system and information exchange is performed through the flask server, using its functions.
