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
    constSize,
	snapshot;

var background = new Image();
var pixelData;

// -------------------MAP TAB--------------------

function blinker() {
  $('.jquery_blink').fadeOut(500);
  $('.jquery_blink').fadeIn(500);
}



function goLocation() {
    $("#nav_btn_go").hide();
    $("map_msg").show();
    $("#map_msg").css("font-size", "140%");
    $("#map_msg").text('Navigating');
    setInterval(blinker, 1000);
    //document.getElementById("map_msg").innerHTML = "Navigating";
}


function getCanvasCoordinates(event) {
    // var x = event.clientX - canvas.getBoundingClientRect().left,
    //     y = event.clientY - canvas.getBoundingClientRect().top;

        var x = event.touches[0].clientX - canvas.getBoundingClientRect().left,
            y = event.touches[0].clientY - canvas.getBoundingClientRect().top;



    return {x: x, y: y};
}

function getCanvasCoordinatesEnd(event) {
    // var x = event.clientX - canvas.getBoundingClientRect().left,
    //     y = event.clientY - canvas.getBoundingClientRect().top;

    var x = event.changedTouches[event.changedTouches.length-1].clientX - canvas.getBoundingClientRect().left,
        y = event.changedTouches[event.changedTouches.length-1].clientY - canvas.getBoundingClientRect().top;


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

        console.log("start drag");
        context.clearRect(0, 0, windowWidth, windowHeight);
        context.drawImage(background,0,0,windowWidth/constSize,windowHeight/constSize);
        dragStartLocation = getCanvasCoordinates(event);
        pixelData = context.getImageData(dragStartLocation.x, dragStartLocation.y, 1, 1).data;
        if((pixelData[0] == 255) && (pixelData[1] == 255) && (pixelData[2] == 255)){
            dragging = true;
            x1 = dragStartLocation.x;
            y1 = dragStartLocation.y;
            takeSnapshot();
        }

}

function drag(event) {
        console.log("drag...");
        if((pixelData[0] == 255) && (pixelData[1] == 255) && (pixelData[2] == 255)){
        var position;
        if (dragging === true) {
            restoreSnapshot();
            position = getCanvasCoordinates(event);
            drawLine(position);
        }
    }


}

function dragStop(event) {
    if((pixelData[0] == 255) && (pixelData[1] == 255) && (pixelData[2] == 255)){
        console.log("end drag");
        if(!chair_moving){
            dragging = false;
            restoreSnapshot();
            var position = getCanvasCoordinatesEnd(event);
            x2 = position.x;
            y2 = position.y;
            drawLine(position);
            var pts = { p1 : [x1*constSize,y1*constSize], p2 : [x2*constSize,y2*constSize] };
            console.log(pts);
            // if (window.confirm("Go to location?")) {
            //     sendPosition();
            // }else{
            //     context.clearRect(0, 0, windowWidth, windowHeight);
            //     context.drawImage(background,0,0,windowWidth/constSize,windowHeight/constSize);
            //
            // }
            //$("#map_msg").hide();
            $("#map_msg").text('');
            $("#nav_btn_go").show();
        }else{
            if(window.confirm("Change location?")){
                //chair stop
                chair_moving = false;
                dragging = false;
                position = getCanvasCoordinatesEnd(event);
                context.clearRect(0, 0, windowWidth, windowHeight);
                context.drawImage(background,0,0,windowWidth/constSize,windowHeight/constSize);
                x2 = position.x;
                y2 = position.y;
                drawLine(position);
                var pts = { p1 : [x1*constSize,y1*constSize], p2 : [x2*constSize,y2*constSize] };
                console.log(pts);
                sendPosition(); // chair start moving
            }
        }
    }


}

function sendPosition() {
    chair_moving = true;

    // Origem  -11.200000, -8.000000, 0.000000
}


function init() {
    canvas = document.getElementById("canvas");


    //background.src = "../ros_ws/maps/iris.png";
    background.src = "static/iris.png";



    windowHeight = 416;
    windowWidth = 608;



    for (var i = 1; i < 100; i=i*1.1) {
        if ((windowWidth/i)<$(window).width()) {
            constSize = i;
            break;
        }
    }

    canvas.height = windowHeight/constSize;
    canvas.width = windowWidth/constSize;

    context = canvas.getContext('2d');
    context.lineWidth = 2;

    background.onload = function(){
        context.drawImage(background,0,0,windowWidth/constSize,windowHeight/constSize);
    }
    // context.background-image = url('static/lol.png');
    // context.background-repeat = no-repeat;

    // canvas.addEventListener('mousedown', dragStart, false);
    // canvas.addEventListener('mousemove', drag, false);
    // canvas.addEventListener('mouseup', dragStop, false);
    canvas.addEventListener('touchstart', dragStart, false);
    canvas.addEventListener('touchmove', drag, false);
    canvas.addEventListener('touchend', dragStop, false);

}

window.addEventListener('load', init, false);
