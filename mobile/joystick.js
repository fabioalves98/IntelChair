var xCenter = window.innerWidth/2;
var yCenter = window.innerHeight/2;
var ros_url = 'localhost';
var ros;



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


manager.on("move", function(event, nipple)
{
    joystick.x = (nipple.position.x - xCenter) * 2;
    joystick.y = (nipple.position.y - yCenter) * 2;
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
		publish_info('/joystick', 'geometry_msgs/Point', point);
	}

}, 50);

function connect(){
	$.get("../api/getip", function(data) {
		var jsondata = $.parseJSON(data);
		ros_url = jsondata.ip;

		ros = new ROSLIB.Ros({
			url : 'ws://' + ros_url + ':9090'
		});

		if(ros_url != 1){
			ros.socket.url = "ws://" + ros_url + ":9090";
		}	
	
		var connected;
		ros_call_service("/connection_service", "intelchair/ChairConnection", {connection: "c"}, function(result){
			console.log("Connection response: " + result.response);
			connected = result.response;
		});	
	
		subscribe_info('/chair_info', 'intelchair/ChairMsg', function(message){
			currentSpeed = message.velocity;
			currentBattery = message.battery;
			setSpeedLabel(currentSpeed);
			setBatteryLabel(currentBattery);
		});
	});
}


function velocityUp(){
	if(currentSpeed < 5){
		currentSpeed++;
		setSpeedLabel(currentSpeed);
		ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "+"}, function(result){
			console.log("Velocity response: " + result.response);
		});
	}
}

function velocityDown(){
	if(currentSpeed > 1){
		currentSpeed--;
		setSpeedLabel(currentSpeed);
		ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "-"}, function(result){
			console.log("Velocity response: " + result.response);
		});
	}
}

function setSpeedLabel(currentSpeed){
	document.getElementById("speed-label").innerHTML = currentSpeed;
}

function setBatteryLabel(currentBattery){
	document.getElementById("battery-label").innerHTML = currentBattery;
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


