


function create_chair_card(cinfo){
	// cinfo.user ="antonio";
	// cinfo.ip ="192.168.1.1";
	var chair_card;
	var chair_card_top = "<div class='col-md-4'> <div class='card shadow mb-4'>\
    <div class='card-header py-3 d-flex flex-row align-items-center justify-content-between'> \
    <h6 class='m-0 font-weight-bold text-primary'>" + cinfo.name + " </h6>"

	var chair_card_used = "<a class='btn btn-primary'>"
	var chair_card_online = "<a class='btn btn-success'>"
	var chair_card_offline = "<a class='btn btn-danger'>"
	var chair_card_navigating = "<a class='btn btn-info'>"
	if(cinfo.status == 'used'){

	}else if(cinfo.status == ){

	}else if(cinfo.status == ){

	}

	var chair_card_body = chair_card_button +  
		"<span class='text-white-100'> \
		</span> \
		<span class='text-white'>" + cinfo.status + " </span> \
		</a> \
		</div> \
		<div class='card-body'> \
		<div class='content-row'> \
		<div class='col-lg-6 text-gray-800'> \
			Battery:  " + cinfo.battery + " \
		</div>";

	var chair_card_ip = " <div class='col-lg-6 text-gray-800'> Ip:  " + cinfo.ip + " </div>";
	if(cinfo.ip != undifined){
		chair_card_body += chair_card_ip;
	}

	var chair_card_end = "</div> \
		</div> \
		</div> \
		</div>";

	chair_card = chair_card_top + chair_card_body + chair_card_end;

  
  $("#wheelchair-content").append(chair_card);

}
function load_chairs(){
	var content = document.getElementById("wheelchair-content");
	$.get("http://localhost:5000/chairs", function(data) {
		var cdata = JSON.parse(data);
		cdata.push(cdata[0]);
		for(var i = 0; i < cdata.length; i++){
			create_chair_card(cdata[i]);
		}
	

	});



}

load_chairs();