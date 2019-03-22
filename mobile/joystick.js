var xCenter = window.innerWidth/2;
var yCenter = window.innerHeight/2;

var options = {
    zone: document.getElementById('zone_joystick'),
    color: 'Blue',
    size: '300',
    position: {left: xCenter, bottom: yCenter},
    mode: 'static'
};
var manager = nipplejs.create(options);
var joystick = {x : 0, y : 0};
var currentSpeed = 1;


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

// ROS INIT
var ros = new ROSLIB.Ros({
    url : 'ws://192.168.1.209:9090'
});

ros.on('connection', function() {
console.log('Connected to websocket server.');
});


subscribe_info('/chair_info', 'intelchair/ChairMsg', function(message){
	currentSpeed = message.velocity;
	setSpeed(currentSpeed);
	console.log(message.battery);
});

setInterval(function(){
    var point = new ROSLIB.Message({
        x: joystick.y,
        y: joystick.x,
        z: 0
    });
	publish_info('/joystick', 'geometry_msgs/Point', point);

}, 50);


function connect(){

	ros_call_service("/connection_service", "intelchair/ChairConnection", {connection: "c"}, function(result){
		console.log("Connection response: " + result.response);
	});
}

function velocityUp(){
	
	if(currentSpeed < 5){
		currentSpeed++;
		setSpeed(currentSpeed);
		ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "+"}, function(result){
			console.log("Velocity response: " + result.response);
		});
	}
	
}

function velocityDown(){
	if(currentSpeed > 1){
		currentSpeed--;
		setSpeed(currentSpeed);
		ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "-"}, function(result){
			console.log("Velocity response: " + result.response);
		});
	}
}

function setSpeed(currentSpeed){
	document.getElementById("speed-label").innerHTML = currentSpeed;
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


