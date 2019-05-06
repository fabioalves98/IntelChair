// Call the dataTables jQuery plugin
$(document).ready(function() {
  $('#dataTable').DataTable();
});

console.log("lol");

$.get( "http://192.168.43.122:5000/chairs", function( data ) {
  console.log(data);
});
