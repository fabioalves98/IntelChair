var ros_url = 'localhost';
var ros;
var chair_connected = false;

var options = {
    zone: document.getElementById('zone_joystick'),
    color: 'Blue',
    size: '100',
    position: {left: '50%', bottom: '50%'},
    mode: 'static'
};
var manager = nipplejs.create(options);
var joystick = {x : 0, y : 0};
var currentSpeed = 1;
var currentBattery;
showIcons();
var start;
var end;


manager.on("move", function(event, nipple)
{
	// Note: The mapping functions expect values ranging from -1 to 1
    joystick.x = (nipple.position.x - window.innerWidth/2) * 2;
	joystick.y = (nipple.position.y - window.innerHeight/2 ) * 2;
	console.log('joystick: ', joystick);
	console.log('wheels: ', map_joystickC(joystick.x, joystick.y));
});

manager.on("end", function(event, nipple)
{
    joystick.x = 0;
    joystick.y = 0;
})

setInterval(function(){
	if(ros){
		var point = new ROSLIB.Message({
			x: joystick.y,
			y: joystick.x,
			z: 0
		});
		// var twist = new ROSLIB.Message({
		// 	linear:{
		// 		x:
		// 		y: 0
		// 		z: 0
		// 	},
		// 	angular:{
		// 		x:
		// 		y:
		// 		z: 0
		// 	}
		// });
		publish_info('/joystick', 'geometry_msgs/Point', point);
		// publish_info('/cmd_vel', 'geometry_msgs/Twist', twist);
	}

}, 50);


function connect(){
	start = + new Date();
	/*
	$.get("/chairs/123123", function(data) {
		console.log(data["data"]);
		var jsondata = $.parseJSON(data["data"]);
		if(jsondata.ip != ""){
			ros_url = jsondata.ip;
		}
*/
		ros = new ROSLIB.Ros({
			// url : 'ws://' + jsondata.ip + ':9090'
			url: 'ws://localhost:9090/'
		});

		if(ros_url != 1){
			ros.socket.url = "ws://" + ros_url + ":9090";
		}
		/*
		var connected = false;
		ros_call_service("/connection_service", "intelchair/ChairConnection", {connection: "c"}, function(result){
			// console.log("Connection response: " + result.response);
			connected = result.response;
			if(connected){
				chair_connected = true;
				document.getElementById("connectBut").innerHTML("Disconnect");
				showIcons();
				subscribe_info('/chair_info', 'intelchair/ChairMsg', function(message){
					currentSpeed = message.velocity;
					currentBattery = message.battery;
					setSpeedLabel(currentSpeed);
					setBatteryLabel(currentBattery);
				});
			}
		});
		*/



	//});
}


function velocityUp(){
	if(currentSpeed < 5){
		ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "+"}, function(result){
			currentSpeed++;
			setSpeedLabel(currentSpeed);
		});

	}
}

function velocityDown(){
	if(currentSpeed > 1){
		ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "-"}, function(result){
			currentSpeed--;
			setSpeedLabel(currentSpeed);
		});
	}
}

function setSpeedLabel(currentSpeed){
	document.getElementById("speed-label").innerHTML = currentSpeed;
}

function setBatteryLabel(currentBattery){
	document.getElementById("battery-label").innerHTML = currentBattery;
}

function showIcons(){
	if(chair_connected){
		$('.icon').css('display','block')
	}else{
		$('.icon').css('display','none')	}
}


/************************* ROS WRAPPER FUNCTIONS ****************************/

function publish_info(topic, msg_type, data){
	var publisher = new ROSLIB.Topic({
		ros: ros,
		name: topic,
		messageType: msg_type

	});

	publisher.publish(data);
}

function subscribe_info(topic, msg_type, callback){
	var listener = new ROSLIB.Topic({
		ros : ros,
		name : topic,
		messageType : msg_type
	});

	listener.subscribe(callback);
}

function ros_call_service(service, service_type, data, callback){
	var _service = new ROSLIB.Service({
		ros: ros,
		name: service,
		serviceType: service_type
	});

	var request = new ROSLIB.ServiceRequest(data);

	_service.callService(request, callback);
}


/********************** Joystick to wheel velocity mapping  *******************/


function map_joystickA(x, y){
	var motor_vel = {R: 0, L: 0};
	motor_vel.R = y - x;
	motor_vel.L = y + x;
	return motor_vel;
}


function ro(x, y){
	return Math.sqrt(x*x + y*y);
}

function theta(x, y){
	return Math.atan(y/x) - Math.PI/2;
	// return Math.atan(y/x);
}


// eixos trocados? x e y?
function map_joystickB(x, y){
	var motor_vel = {R: 0, L: 0};
	function right(x, y){
		var t = theta(x, y);
		var r = ro(x,y);
		if(t >= (-Math.PI/2) && t <= 0) return r;
		else if(t >= (Math.PI/2) && t <= Math.PI) return -r;
		else if((t > 0 && t < (Math.PI / 2)) || (t > -Math.PI && t < -(Math.PI/2))) return ((r * (Math.cos(t + (Math.PI/4)))) / Math.cos((-Math.PI / 4)))
	}

	function left(x, y){
		var t = theta(x, y);
		var r = ro(x,y);
		if(t >= 0 && t <= (-Math.PI/2)) return r;
		else if(t >= -Math.PI && t <= (-Math.PI/2)) return -r;
		else if((t > (-Math.PI / 2) && t < 0) || (t > (Math.PI/2) && t < Math.PI)) return ((r * (Math.cos(t - (Math.PI/4)))) / Math.cos((-Math.PI / 4)))

	}

	motor_vel.R = right(x, y);
	motor_vel.L = left(x, y);
	return motor_vel;


}

function map_joystickC(x, y){
	x = x / 100;
	y = y / 100;
	var motor_vel = {R: 0, L: 0};
	function nx(x){
		if(x/2 > 0.1) return 0.1+((x/2) - 0.1)/2;
		else if(x/2 < -0.1) return -0.1+((x/2) + 0.1)/2;
		else return x/2;
	}

	motor_vel.R = y - nx(x);
	motor_vel.L = y + nx(x);
	return motor_vel;
}
