var options = {
    zone: document.getElementById('zone_joystick'),
    color: 'Blue',
    size: '100',
    position: {left: '50%', bottom: '50%'},
    mode: 'static'
};
var manager = nipplejs.create(options);
var joystick = {x : 0, y : 0};


manager.on("move", function(event, nipple)
{
    joystick.x = (nipple.position.x - window.innerWidth/2) * 2;
	joystick.y = (nipple.position.y - window.innerHeight/2 ) * -2;
	console.log('joystick: ', joystick);
});

manager.on("end", function(event, nipple)
{
    joystick.x = 0;
    joystick.y = 0;
})

setInterval(function()
{
	if(ros)
	{
		var point = new ROSLIB.Message(
		{
			x: joystick.y,
			y: joystick.x,
			z: 0
		});

		publish_info('/joystick', 'geometry_msgs/Point', point);
	}
}, 50);

