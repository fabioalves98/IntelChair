var chair_connected = 0;
var chair_ip;
var chair_moving = false;
var currentSpeed = 1;
var currentBattery;
var start;
var username;
var end;
var manual_control = true;

showIcons();

$('#usershow').html(localStorage.username);

$(document).ready(function()
{
	$.get( '/users/' + localStorage.username , function( data )
    {
		var user = JSON.parse(data);
		if(user == undefined) {
            window.location.replace("/login");
		}
		if (user['role'] == 'guest'){
			$('#mappingTab').hide();
		}
	});

	$(window).bind("beforeunload", function () 
	{
		disconnect();
	})
});

//setInterval(post_chair_info, 30000);

function post_chair_info()
{
    if(chair_connected)
    {
		$.ajax(
        { 
			url: '/chairs/123213',
			type: 'PUT',
            data : 
            {
				'status'    : 'Taken',
                'user'  	: localStorage.username
            },
			success: function() {}
		});
	}
}

function connect()
{
	start = Math.floor(Date.now() / 1000)
	console.log(start);

	$.get("/chairs/123123", function(data) 
	{
		var jsondata = $.parseJSON(data);
		if(jsondata.ip != "")
		{
			ros_url = jsondata.ip;
			chair_ip = jsondata.ip;

		}
		ros = new ROSLIB.Ros(
		{
			url : 'ws://' + jsondata.ip + ':9090'
			//url: 'ws://localhost:9090/'
		});
		if(ros_url != 1)
		{
			ros.socket.url = "ws://" + ros_url + ":9090";
		}
		ros.on('connection', function() 
		{
			console.log('Connected to websocket server.');
			chair_connected = 1;
			showIcons();
			subscribe_info('/chair_info', 'intelchair/ChairMsg', function(message)
			{
				currentSpeed = message.velocity;
				currentBattery = message.battery;
				setSpeedLabel(currentSpeed);
				setBatteryLabel(currentBattery);
			});

			publish_info('/chair_control', 'intelchair/ChairMsg', new ROSLIB.Message(
			{
				velocity: currentSpeed,
				battery: currentBattery,
				connected: chair_connected
			}));

			$.ajax(
			{
				url 	: 	'/chairs/123123',
				type 	: 	'PUT',
				data	:	
				{
					"user"  	: localStorage.username,
					"status"	: "Taken"
				},
				success : function(data)
				{
					console.log(data)
				}
			});

		});

		ros.on('error', function(error) {
			console.log('Error connecting to websocket server');
			alert('Cant connect');
		});

		ros.on('disconnect', function(){
			console.log('Disconnected');
			alert('disconnect');
		});

		ros.on("close", function(){
			disconnect();
			alert("closed");	
		})
	});
}

function disconnect(){	

	if(chair_connected == 1){
		$.ajax(
		{
			url 	: 	'/chairs/123123',
			type 	: 	'PUT',
			data	:	
			{
				"user"		: null,
				"status"	: "Online"
			},
			success : function(data)
			{
				console.log(data)
			}
		});

		end = Math.floor(Date.now() / 1000)

		$.post('/history',
		{
			'startTime'	: start,
			'endTime' 	: end,
			'username'	: localStorage.username,
			'chair'		: '123123'
		},
		function(data, status)
		{
			console.log(status)
		});
	}

	$.post('/logout',
	{
		'username' : localStorage.username
	},
	function(data, status) 
	{
		console.log(status);
    });
	
	chair_connected = 0;
    localStorage.clear();
	window.location.replace("/login");
}

function velocityUp()
{
	publish_info("/chair_control", "intelchair/ChairMsg", new ROSLIB.Message(
	{
		velocity: currentSpeed + 1,
		battery: currentBattery,
		connected: chair_connected
	}));
}

function velocityDown()
{
	publish_info("/chair_control", "intelchair/ChairMsg", new ROSLIB.Message(
	{
		velocity: currentSpeed - 1,
		battery: currentBattery,
		connected: 1
	}));
}

function setSpeedLabel(currentSpeed)
{
	document.getElementById("speed-label").innerHTML = currentSpeed;
}

function setBatteryLabel(currentBattery)
{
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