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
    originX=0,
    originY=0,
	snapshot;

var background = new Image();


// -------------------MAP TAB--------------------

function zoomIn(){
    constSize = constSize*0.8;
    console.log(constSize);
    background.src = "static/lol.png";
    background.onload = function(){
        context.clearRect(0, 0, window.innerWidth, window.innerHeight);
        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

function moveR(){
    originX = originX + 20;
    background.src = "static/lol.png";
    background.onload = function(){
        context.clearRect(0, 0, window.innerWidth, window.innerHeight);
        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

function moveL(){
    originX = originX - 20;
    background.src = "static/lol.png";
    background.onload = function(){
        context.clearRect(0, 0, window.innerWidth, window.innerHeight);
        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

function moveU(){
    originY = originY - 20;
    background.src = "static/lol.png";
    background.onload = function(){
        context.clearRect(0, 0, window.innerWidth, window.innerHeight);
        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

function moveD(){
    originY = originY + 20;
    background.src = "static/lol.png";
    background.onload = function(){
        context.clearRect(0, 0, window.innerWidth, window.innerHeight);
        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

function zoomOut(){
    constSize = constSize*1.2;
    console.log(constSize);
    background.src = "static/lol.png";
    background.onload = function(){
        context.clearRect(0, 0, window.innerWidth, window.innerHeight);

        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

function getCanvasCoordinates(event) {
    var x = event.clientX - canvas.getBoundingClientRect().left + originX,
        y = event.clientY - canvas.getBoundingClientRect().top + originY;

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
        var pts = { p1 : [x1*constSize,y1*constSize], p2 : [x2*constSize,y2*constSize] };
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
            context.clearRect(0, 0, window.innerWidth, window.innerHeight);
            context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
            x2 = position.x;
            y2 = position.y;
            drawLine(position);
            var pts = { p1 : [x1*constSize,y1*constSize], p2 : [x2*constSize,y2*constSize] };
            console.log(pts);
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

    background.src = "static/lol.png";




    windowHeight = 640;
    windowWidth = 544;
    canvas.height = windowHeight;
    canvas.width = windowWidth;


    for (var i = 1; i < 100; i++) {
        if ((windowWidth/i)<$(window).width()) {
            constSize = i;
            break;
        }
    }

    context = canvas.getContext('2d');
    context.lineWidth = 2;

    background.onload = function(){
        context.drawImage(background,originX,originY,windowWidth/constSize,windowHeight/constSize);
    }
    // context.background-image = url('static/lol.png');
    // context.background-repeat = no-repeat;

    canvas.addEventListener('mousedown', dragStart, false);
    canvas.addEventListener('mousemove', drag, false);
    canvas.addEventListener('mouseup', dragStop, false);
}

window.addEventListener('load', init, false);
