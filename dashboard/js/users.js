$(document).ready(function() 
{
    var table = $('#user_table').DataTable();

    $.get( 'http://localhost:5000/users', function( data ) 
    {
        var users = JSON.parse(data);
        
        users.forEach(element => 
            {
            table.row.add ( [
                element['firstname'] + " " + element['lastname'],
                element['username'],
                element['email'],
                element['age']
            ] ).draw( false );
        });
    });
});

$('#add_user').click( function() 
{
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
    function(data, status)
    {
        console.log(status);
    });

    $('#add_modal').modal('toggle');
 	
 	location.reload();
})

$('#update_user').click( function() 
{
    $.post( 'http://localhost:5000/users/' + $('#up_un').val(),
    {
        'first-name'    : $('#up_fn').val(),
        'last-name'     : $('#up_ln').val(),
        'username'      : $('#up_un').val(),
        'password'      : $('#up_pw').val(),
        'email'         : $('#up_em').val(),
        'age'           : $('#up_ag').val(),
        'gender'        : $('#up_ge').val()
    },
    function(data, status)
    {
        console.log(status);
    });
    
    location.reload();
})

$('#rem_user').click( function() 
{
    $.post( 'http://localhost:5000/remove/users/' + $('#rem_un').val(),
    function(data, status)
    {
        console.log(status);
    });

    $('#rem_modal').modal('toggle');

    location.reload();
})


$('#search_user').click( function() 
{
    $.get( 'http://localhost:5000/users/' + $('#srch_un').val(),
    function(data, status)
    {   
        var user = JSON.parse(data);
        
        if(user == undefined){
            alert("User not found");
            location.reload();
        }

        $('#up_fn').val(user['firstname']),
        $('#up_ln').val(user['lastname']),
        $('#up_un').val(user['username']),
        console.log(user['password']);
        $('#up_pw').val(user['password']),
        $('#up_em').val(user['email']),
        $('#up_ag').val(user['age']),
        $('#up_ge').val(user['gender'])

        console.log(status);
    });

    $('#search_modal').modal('hide');
    $('#update_modal').modal('show');
      
})
