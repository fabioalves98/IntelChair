// Call the dataTables jQuery plugin
$(document).ready(function() {
  var table = $('#dataTable').DataTable();

  $.get( "http://localhost:5000/users", function( data ) {
    
    var users = JSON.parse(data);

    users.forEach(element => {
      table.row.add ( [
        element["first-name"] + " " + element["last-name"],
        element["username"],
        element["email"],
        element["age"]
      ] ).draw( false );
    });

  });

});



