var ros_url = 'localhost';
var ros;
var chair_connected = 0;
var chair_ip;
var chair_moving = false;

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
var username;
var end;

var canvas,
    context,
    x1,
    y1,
    x2,
    y2,
    dragging = false,
    dragStartLocation,
    windowHeight,
    windowWidth,
    snapshot;


// -------------------MAP TAB--------------------

function getCanvasCoordinates(event) {
    var x = event.clientX - canvas.getBoundingClientRect().left,
        y = event.clientY - canvas.getBoundingClientRect().top;

    return {x: x, y: y};
}

function takeSnapshot() {
    snapshot = context.getImageData(0, 0, canvas.width, canvas.height);
}

function restoreSnapshot() {
    context.putImageData(snapshot, 0, 0);
}


function drawLine(position) {
    var angle = Math.atan2(position.y - dragStartLocation.y, position.x - dragStartLocation.x);
    var maxY = dragStartLocation.y + Math.sin(angle) * 30;
    var maxX = dragStartLocation.x + Math.cos(angle) * 30;
    context.beginPath();
    context.moveTo(dragStartLocation.x, dragStartLocation.y);
    // context.lineTo(position.x, position.y);
    context.lineTo(maxX, maxY);
    context.stroke();
    context.beginPath();
    context.arc(x1, y1, 3, 0, 2 * Math.PI);
    context.stroke();
}

function dragStart(event) {
        dragging = true;
        dragStartLocation = getCanvasCoordinates(event);
        x1 = dragStartLocation.x;
        y1 = dragStartLocation.y;
        takeSnapshot();

}

function drag(event) {
        var position;
        if (dragging === true) {
            restoreSnapshot();
            position = getCanvasCoordinates(event);
            drawLine(position);
        }

}

function dragStop(event) {
    if(!chair_moving){
        dragging = false;
        restoreSnapshot();
        var position = getCanvasCoordinates(event);
        x2 = position.x;
        y2 = position.y;
        drawLine(position);
        var pts = { p1 : [x1,y1], p2 : [x2,y2] };
        console.log(pts);
        //context.clearRect(0, 0, windowWidth, windowHeight);
        if (window.confirm("Go to location?")) {
            sendPosition();
        }
    }else{
        if(window.confirm("Change location?")){
            //chair stop
            chair_moving = false;
            dragging = false;
            position = getCanvasCoordinates(event);
            context.clearRect(0, 0, windowWidth, windowHeight);
            x2 = position.x;
            y2 = position.y;
            drawLine(position);
            sendPosition(); // chair start moving
        }
    }

}

function sendPosition() {
    chair_moving = true;

    // Origem  -11.200000, -8.000000, 0.000000
}

function init() {
    canvas = document.getElementById("canvas");

    windowHeight = 640;
    windowWidth = 544;
    canvas.height = windowHeight;
    canvas.width = windowWidth;

    context = canvas.getContext('2d');
    context.lineWidth = 2;

    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

window.addEventListener('load', init, false);

//-----------------------------

$('#usershow').html(localStorage.username);

$(document).ready(function()
{
	$.get( 'http://localhost:5000/users/' + localStorage.username , function( data )
    {
		var user = JSON.parse(data);
		if(user == undefined) {
            alert("User not found");
		}
		if (user['role'] == 'null'){
			$('#mappingTab').hide();
		}
	});
});

manager.on("move", function(event, nipple)
{
	// Note: The mapping functions expect values ranging from -1 to 1
    joystick.x = (nipple.position.x - window.innerWidth/2) * 2;
	joystick.y = (nipple.position.y - window.innerHeight/2 ) * -2;
	console.log('joystick: ', joystick);
	console.log('wheels: ', map_joystickC(joystick.x, joystick.y));
});

manager.on("end", function(event, nipple)
{
    joystick.x = 0;
    joystick.y = 0;
})

function storeUser(){
	localStorage.setItem('username', $('#user').val())
}

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

//Each 30 secs, post chair info to the server
setInterval(post_chair_info, 30000);

function post_chair_info(){
	if(chair_connected){
		$.post('/chair/123123',
			{
				'username' 	: localStorage.username,
				'status'  	: 'Taken',
				'battery'	: currentBattery,
				'chair_ip'		: chair_ip

			},function(data, status){
				console.log(status);
		});
	}
}

function connect(){
	// var s = new Date();
	// start = s.getDate()+'/'+(s.getMonth()+1)+'/'+s.getFullYear()+"-"+s.getHours()+":"+s.getMinutes();
	start = new Date().getTime();
	$.get("/chairs/123123", function(data) {
		var jsondata = $.parseJSON(data);
		if(jsondata.ip != ""){
			ros_url = jsondata.ip;
			chair_ip = jsondata.ip;

		}

		console.log(jsondata);

		ros = new ROSLIB.Ros({
			//url : 'ws://' + jsondata.ip + ':9090'
			url: 'ws://localhost:9090/'
		});
		if(ros_url != 1){
			ros.socket.url = "ws://" + ros_url + ":9090";
		}
		ros.on('connection', function() {
			console.log('Connected to websocket server.');
			chair_connected = 1;
			showIcons();
			subscribe_info('/chair_info', 'intelchair/ChairMsg', function(message){
				currentSpeed = message.velocity;
				currentBattery = message.battery;
				setSpeedLabel(currentSpeed);
				setBatteryLabel(currentBattery);
			});

			publish_info('/chair_info_control', 'intelchair/ChairMsg', new ROSLIB.Message({
				velocity: currentSpeed,
				battery: currentBattery,
				connected: chair_connected
			}));

		});

		$.post('/chair/123123',
		{
			'username' 	: localStorage.username,
			'status'  	: 'Taken',
			'battery'	: currentBattery,
			'chair_ip'	: chair_ip
		},
		function(data, status){
			console.log(status);
		});

		$.post('/users/chair/' + localStorage.username,
		{
			'chair_user' 	: '123123'
		},
		function(data, status){
			console.log(status);
		});

		ros.on('error', function(error) {
			console.log('Error connecting to websocket server');
			alert('Cant connect');
		});

		ros.on('disconnect', function(){
			console.log('Disconnected');
			disconnect();
		})
	});

}

function disconnect(){
	chair_connected = 0;
	// var s = new Date();
	// end = s.getDate()+'/'+(s.getMonth()+1)+'/'+s.getFullYear()+"-"+s.getHours()+":"+s.getMinutes();
	$.post( 'http://localhost:5000/chair/123123',
	{
		'username' 	: "None",
		'status'  	: 'Online',
		'chair_ip'		: chair_ip,
		'battery'	: currentBattery
	},
	function(data, status){
		console.log(status);
	});

	$.post( 'http://localhost:5000/users/chair/' + localStorage.username,
	{
		'chair_user' 	: null
	},
	function(data, status){
		console.log(status);
	});

	end = new Date().getTime();
	$.post( 'http://localhost:5000/history',
	{
		'startTime'	: start,
		'endTime' 	: end,
		'username'	: localStorage.username,
		'chairID'	: '123123'
	},
	function(data, status)
	{
		console.log(status)
		window.location.replace("http://localhost:5000/login");
	});

	location.reload();
}

function velocityUp(){
	// if(currentSpeed < 5){
		// ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "+"}, function(result){
		// 	currentSpeed++;
		// 	setSpeedLabel(currentSpeed);
		// });
		publish_info("/chair_info_control", "intelchair/ChairMsg", new ROSLIB.Message({
			velocity: currentSpeed + 1,
			battery: currentBattery,
			connected: chair_connected
		}));

	// }
}

function velocityDown(){
	// if(currentSpeed > 1){
		// ros_call_service("/velocity_service", "intelchair/ChairVelocity", {velocity: "-"}, function(result){
		// 	currentSpeed--;
		// 	setSpeedLabel(currentSpeed);
		// });
		publish_info("/chair_info_control", "intelchair/ChairMsg", new ROSLIB.Message({
			velocity: currentSpeed - 1,
			battery: currentBattery,
			connected: 1
		}));
	// }
}

function setSpeedLabel(currentSpeed){
	document.getElementById("speed-label").innerHTML = currentSpeed;
}

function setBatteryLabel(currentBattery){
	document.getElementById("battery-label").innerHTML = currentBattery;
}

function showIcons(){
	if(chair_connected){
		$('.icon').show();
		$('.spd-btn').show();
		$('#btn-disconnect').show();
		$('#btn-connect').hide();
		$('#zone_joystick').show();
		$('#zone_joystick_message').hide();

	}else{
		$('.icon').hide();
		$('.spd-btn').hide();
		$('#btn-disconnect').hide();
		$('#btn-connect').show();
		$('#zone_joystick').hide();
		$('#zone_joystick_message').show();
	}
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

