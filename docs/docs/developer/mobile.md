#Mobile Application
The mobile application is built using HTML, CSS, JavaScript and Ionic. It allows the user to interact with the system by providing features such as manual control and navigation. The app interacts  with the wheelchair through [Roslibjs](http://wiki.ros.org/roslibjs), a javascript library that provides publishing, subscribing and other tools to connect to ROS using WebSockets via rosbridge.

Apart from the front-end, in terms of communication with the chair, we use functions that work as a set of wrapper methods, that simplify the interation with the ROS system. The following function is an example of that wrapper method, that receives the topic's name and type as well as the data the caller wants to publish in the previously mentioned topic.

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

In addition to roslibjs, we used a library called [nipplejs](https://github.com/yoannmoinet/nipplejs)  create the virtual joystick that allows the manual control of the chair. Finally all access to information about the system and information exchange is performed through the flask server, using its functions.
