var xCenter = window.innerWidth/2;
var yCenter = window.innerHeight/2;
console.log(xCenter + '  ' + yCenter);

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
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
console.log('Connected to websocket server.');
});

// var pub = new ROSLIB.Topic({
//     ros : ros,
//     name : '/joystick',
//     messageType : 'std_msgs/String'
// });

var pub_geometry = new ROSLIB.Topic({
	ros: ros,
	name: '/joystick',
	messageType: 'geometry_msgs/Point'
});

setInterval(function(){
	// if (touch == true)
	// {
		console.log('\n');
		console.log('X : ' + joystick.deltaX());
		console.log('Y : ' + joystick.deltaY());
		console.log('\n');

		var coord = "X: " + joystick.deltaX() + " | Y: " + joystick.deltaY();

		var point = new ROSLIB.Message({
			x: joystick.deltaX(),
			y: joystick.deltaY(),
			z: 0
		  });
		pub_geometry.publish(point);
	// }
}, 300);