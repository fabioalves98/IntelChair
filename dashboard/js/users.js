// Call the dataTables jQuery plugin
$(document).ready(function() {
    var table = $('#user_table').DataTable();
    
    $.get( 'http://localhost:5000/users', function( data ) {
        var users = JSON.parse(data);
        
        users.forEach(element => {
            table.row.add ( [
                element['first-name'] + " " + element['last-name'],
                element['username'],
                element['email'],
                element['age']
            ] ).draw( false );
        });
    });
});

$('#add_user').click( function() {
    $.post( 'http://localhost:5000/users',
    {
        'first-name'    : $('#add_fn').val(),
        'last-name'     : $('#add_ln').val(),
        'username'      : $('#add_un').val(),
        'password'      : $('#add_pw').val(),
        'email'         : $('#add_em').val(),
        'age'           : $('#add_ag').val(),
        'gender'        : $('#add_ge').val()
    },
    function(data, status){
        console.log(status);
    });

    $('#add_modal').modal('toggle');
    
    location.reload();
})

$('#rem_user').click( function() {
    $.ajax({
        url: 'http://localhost:5000/users/' + $('#rem_un').val(),
        type: 'DELETE',
        success: function(data, status) {
            console.log(status);
        }
    })

    $('#rem_modal').modal('toggle');

    location.reload();
})



