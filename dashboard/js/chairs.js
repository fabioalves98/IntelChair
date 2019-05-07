var active_card = null;
var cdata = null;

function create_chair_card(cinfo){
	// cinfo.user ="antonio";
	// cinfo.ip ="192.168.1.1";
	if(cinfo.status == undefined) cinfo.status = "Offline";
	var chair_card;
	var chair_card_top = "<div onclick=\"set_active_card('" + cinfo.name + "')\" > <div class='card shadow mb-4'>\
	<div id='" + cinfo.name + "' class='card-header py-3 d-flex flex-row align-items-center justify-content-between'> \
	<h6 class='m-0 font-weight-bold text-primary'>" + cinfo.name + " </h6>";
	
	

	var chair_card_used = "<a class='btn btn-sm btn-primary'>";
	var chair_card_online = "<a class='btn btn-sm btn-success'>";
	var chair_card_offline = "<a class='btn btn-sm  btn-danger'>";
	var chair_card_navigating = "<a class='btn btn-sm btn-info'>";
	var chair_card_button;
	if(cinfo.status == 'Taken'){
		chair_card_button = chair_card_used;
	}else if(cinfo.status == 'Online'){
		chair_card_button = chair_card_online;
	}else if(cinfo.status == 'Navigating'){
		chair_card_button = chair_card_navigating;
	}else{
		chair_card_button = chair_card_offline;
	}

	var chair_card_body = chair_card_button +  "<span class='text-white-100'> \
		</span> \
		<span class='text-white'>" + cinfo.status + " </span> \
		</a> \
		</div> ";
	
	var chair_card_body_online = chair_card_body + "<div class='card-body'> \
	<div class='content-row'>";
	if(cinfo.status == 'Online' || cinfo.status == 'Taken' || cinfo.status == 'Navigating') chair_card_body = chair_card_body_online;

	
	var chair_card_battery= "<div class='row no-gutters align-items-center'>\
		<div class='col mr-2'>\
		<div class='text-x1s font-weight-bold text-info text-uppercase mb-1'>Battery</div>\
		<div class='row no-gutters align-items-center'>\
		<div class='col-auto'>\
			<div class='h5 mb-0 mr-3 font-weight-bold text-gray-800'>" + cinfo.battery*10 + "%</div>\
		</div>\
		<div class='col'>\
			<div class='progress progress-sm mr-2'>\
			<div class='progress-bar bg-info' role='progressbar' style='width: " + cinfo.battery*10+"%' aria-valuenow='50' aria-valuemin='0' aria-valuemax='100'></div>\
			</div>\
		</div>\
		</div>\
		</div>\
		<div class='col-auto'>\
		<i class='fas fa-battery-full fa-2x fa-rotate-270'></i>\
		</div>\
		</div>\
		</div>";


	var chair_card_ip = "<div class='row'><div class='col-md-6'><div class='text-x1s font-weight-bold text-info mt-3 text-uppercase mb-1'>IP</div>\
	<div class='h6 mb-0 mr-3 font-weight-bold text-gray-800'>" +cinfo.ip +"</div></div>";
	var chair_card_user = "<div class='col-md-6'><div class='text-x1s font-weight-bold text-info mt-3 text-uppercase mb-1'>USER</div>\
	<div class='h6 mb-0 mr-3 font-weight-bold text-gray-800'>" +cinfo.user +"</div></div></div>";

	
	if(cinfo.battery != undefined) chair_card_body += chair_card_battery;
	if(cinfo.ip != undefined) chair_card_body += chair_card_ip;
	if(cinfo.user != undefined) chair_card_body += chair_card_user;

	var chair_card_end = "</div> \
		</div> \
		</div> \
		</div>";

	chair_card = chair_card_top + chair_card_body + chair_card_end;

  
  $("#wheelchair-content").append(chair_card);

}



function set_info_card(){
	document.getElementById("cname").innerHTML = active_card.name;
	document.getElementById("ccompany").innerHTML = active_card.company;
	document.getElementById("cmodel").innerHTML = active_card.model;
	document.getElementById("cid").innerHTML = active_card.id;
	
	if(active_card.user != undefined){
		document.getElementById("no-user").style.display = "none";
		document.getElementById("disconnect-user").style.display = "block";
	}else{
		document.getElementById("no-user").style.display = "block";
		document.getElementById("disconnect-user").style.display = "none"; 
	}
	
	
	
}

function set_active_card(chair_name){
	for (var i=0; i < cdata.length; i++) {
		if (cdata[i].name === chair_name) {
			active_card = cdata[i];
        }
	}
	set_info_card();
	for(var i = 0; i < cdata.length; i++){
		if(active_card.name == cdata[i].name){
			document.getElementById(cdata[i].name).style.background = '#b9ff9b';
		}else{
			document.getElementById(cdata[i].name).style.background = '';
		}
	}

	// load_chairs();

}

function add_chair_card(){

	var card = "<div class='card shadow mb-4'> \
		<div style='cursor:pointer' onclick='new_chair()' align='center' class='card-body'>\
		<i class='fas fa-plus fa-2x'></i> \
		</div> \
		</div> \
		</div> ";

	$("#wheelchair-content").append(card);
}

function new_chair(){
	$('#add_chair_modal').modal('toggle');
}
function add_chair(){
	$.post('http://localhost:5000/chairs',
    {
		'company'    	: $('#add_c').val(),
		'model'  	    : $('#add_m').val(),
		'name'     		: $('#add_n').val(),
		'id'					: $('#add_i').val(),
		'status' 	    : 'Offline'
    },
    function(data, status){
        console.log(status);
    });

	$('#add_chair_modal').modal('toggle');
	load_chairs();
}


function remove_chair(){
	$.ajax({
        url: 'http://localhost:5000/chairs/' + active_card.name,
        type: 'DELETE',
        success: function(data, status) {
            console.log(status);
        }
	});
	active_card = null;
	load_chairs();
	
}


function load_chairs(){
	
	$('#wheelchair-content').empty();	

	$.get("http://localhost:5000/chairs", function(data) {
		cdata = JSON.parse(data);
		
		for(var i = 0; i < cdata.length; i++){
			create_chair_card(cdata[i]);
		}
		if(active_card == null) set_active_card(cdata[0].name);

		set_info_card();
		add_chair_card();

	});
}

load_chairs();