var voice_active = false;

if (annyang) {
    var commands = {
      'connect': function(){
        if (chair_connected == 0){
            connect();
        }
      },
      'exit': disconnect,
      'voice off': voice_activate,
      'speed up': function(){
        if (chair_connected == 1){
            velocityUp();
        }
        },
      'speed down': function(){
        if (chair_connected == 1){
            velocityDown();
        }
    },
      'maps': function(){
        $('#tab-button-settings').trigger("click");
      },
    
      'mapping': function(){
        $('#tab-button-mapping').trigger("click");
      },
      'navigation': function(){
        $('#tab-button-home').trigger("click");
      },
      'zoom in': function(){
        zoomIn();
      },
      'zoom out' : function(){
        zoomOut();
      },
      'right' : function(){
        moveR();
      },
      'left' : function(){
        moveL();
      },
      'up' : function(){
        moveU();
      },
      'down' : function(){
        moveD();
      }
    };
  
    annyang.addCommands(commands);
}

function voice_activate(){
	if (voice_active == false){
        voice_active = true;
        $("#voice_ico").attr("name","mic-off");
        annyang.start();
		alert('voice commands activated');
	}
    else{
        $("#voice_ico").attr("name","mic");
        voice_active = false;
        annyang.abort();
        alert('voice commands deactivated');
    } 	
}
