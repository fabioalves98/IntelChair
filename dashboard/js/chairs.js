var active_card = null;
var cdata = null;

function create_chair_card(cinfo){
	// cinfo.user ="antonio";
	// cinfo.ip ="192.168.1.1";
	if(cinfo.status == undefined) cinfo.status = "Offline";
	var chair_card;
	var chair_card_top = "<div onclick=\"set_active_card('" + cinfo.name + "')\" id='" + cinfo.name + "'> <div class='card shadow mb-4'>\
    <div class='card-header py-3 d-flex flex-row align-items-center justify-content-between'> \
    <h6 class='m-0 font-weight-bold text-primary'>" + cinfo.name + " </h6>"

	var chair_card_used = "<a class='btn btn-primary'>";
	var chair_card_online = "<a class='btn btn-success'>";
	var chair_card_offline = "<a class='btn btn-danger'>";
	var chair_card_navigating = "<a class='btn btn-info'>";
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
	if(cinfo.status == 'Online') chair_card_body = chair_card_body_online;

	
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
	<div class='h5 mb-0 mr-3 font-weight-bold text-gray-800'>" +cinfo.ip +"</div></div>";
	var chair_card_user = "<div class='col-md-6'><div class='text-x1s font-weight-bold text-info mt-3 text-uppercase mb-1'>USER</div>\
	<div class='h5 mb-0 mr-3 font-weight-bold text-gray-800'>" +cinfo.user +"</div></div></div>";

	
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


function create_info_card(){
	console.log(active_card);
	// $("#detail-content").empty();
	// var card = "<div class='card shadow mb-4'> \
	// 	<div class='card-header py-3'> \
	// 	<h6 class='m-0 font-weight-bold text-primary'>" + active_card.name + "</h6> \
	// 	</div> \
	// 	<div class='card-body'> \
	// 	<div class='text-center'> \
	// 	<img class='img-fluid px-3 px-sm-4 mt-3 mb-4' style='width: 25rem;' src='img/undraw_posting_photo.svg'> \
	// 	</div> \
	// 	<p>Add some quality, svg illustrations to your project courtesy of  images that you can use completely free and without attribution!</p> \
	// 	<a target='_blank' rel='nofollow' href='https://undraw.co/'>Browse Illustrations on unDraw &rarr;</a> \
	// 	</div> \
	// 	</div>";
	
	// $("#detail-content").append(card);

	
}

function set_active_card(chair_name){
	for (var i=0; i < cdata.length; i++) {
		if (cdata[i].name === chair_name) {
			active_card = cdata[i];
        }
	}
	create_info_card();

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
		'id'			: 'albc4',
		'status' 	    : 'Offline',
    },
    function(data, status){
        console.log(status);
    });

	$('#add_chair_modal').modal('toggle');
	load_chairs();
}


function remove_chair(){
	$.ajax({
        url: 'http://localhost:5000/chairs/' + 'AChair3',
        type: 'DELETE',
        success: function(data, status) {
            console.log(status);
        }
    })
}


function load_chairs(){
	
	$('#wheelchair-content').empty();	

	$.get("http://localhost:5000/chairs", function(data) {
		cdata = JSON.parse(data);
		for(var i = 0; i < cdata.length; i++){
			create_chair_card(cdata[i]);
		}
		if(active_card == null) active_card = cdata[0];
		create_info_card();
		add_chair_card();

	});
}

load_chairs();