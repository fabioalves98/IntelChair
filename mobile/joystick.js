var xCenter = window.innerWidth/2;
var yCenter = window.innerHeight/2;
console.log(xCenter + '  ' + yCenter);

// Has to be a subscriber listening for the speed
var currentSpeed = 1;


setSpeed(currentSpeed);
console.log(currentSpeed);


var joystick = new VirtualJoystick({
		mouseSupport: true,
		stationaryBase: true,
		baseX: xCenter,
		baseY: yCenter,
		limitStickTravel: true,
		stickRadius: 300
	});

var touch = false;

joystick.addEventListener('touchStart', function(){
	touch = true;
	console.log('start - touch')
})

joystick.addEventListener('mousedown', function(){
	touch = true;
	console.log('start - mouse')
})

joystick.addEventListener('touchEnd', function(){
	touch = false;
	console.log('stop - touch')
})

joystick.addEventListener('mouseup', function(){
	touch = false;
	console.log('stop - mouse')
})

// ROS INIT
var ros = new ROSLIB.Ros({
    url : 'ws://192.168.1.209:9090'
});

ros.on('connection', function() {
console.log('Connected to websocket server.');
});

var pub_geometry = new ROSLIB.Topic({
	ros: ros,
	name: '/joystick',
	messageType: 'geometry_msgs/Point'
});

setInterval(function(){
	if (touch == true)
	{
		var point = new ROSLIB.Message({
			x: joystick.deltaY(),
			y: joystick.deltaX(),
			z: 0
	  	});
		pub_geometry.publish(point);
	}
	else
	{
		var point = new ROSLIB.Message({
			x: 0,
			y: 0,
			z: 0
	  	});
		pub_geometry.publish(point);
	}
}, 50);


function connect(){
	console.log("Sending connection msg");
	publish_info("/connection", "std_msgs/String", {data: "connect"});
}

function velocityUp(){
	
	if(currentSpeed < 5){
		currentSpeed++;
		setSpeed(currentSpeed);
		publish_info("/max_speed", "std_msgs/String", {data: "+"});
	}
	
}

function velocityDown(){
	if(currentSpeed > 1){
		currentSpeed--;
		setSpeed(currentSpeed);
		publish_info("/max_speed", "std_msgs/String", {data: "-"});
	}
}

function setSpeed(currentSpeed){
	document.getElementById("speed-label").innerHTML = currentSpeed;
}


function publish_info(topic, msg_type, data){
	var publisher = new ROSLIB.Topic({
		ros: ros,
		name: topic,
		messageType: msg_type
		
	});

	publisher.publish(data);
}