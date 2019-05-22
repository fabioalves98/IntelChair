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