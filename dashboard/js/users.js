ip = 'localhost:5000'
ip2 = '192.168.43.122:5000'
url = 'http://' + ip2

$(document).ready(function() 
{
    var table = $('#user_table').DataTable();
    $.get( url +'/users', function( data ) 
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
    var role = 'guest';
    if($('#add_rl').is(':checked')){
        role = 'admin';
    }
    
    $.post( url +'/users',
    {
        'firstname'    : $('#add_fn').val(),
        'lastname'     : $('#add_ln').val(),
        'username'      : $('#add_un').val(),
        'password'      : $('#add_pw').val(),
        'email'         : $('#add_em').val(),
        'age'           : $('#add_ag').val(),
        'role'          : role
    },
    function(data, status)
    {
        console.log(status);
        location.reload();

    });


    $('#add_modal').modal('toggle');

});

$('#update_user').click( function() 
{
    var role = 'guest';
    if($('#up_rl').is(':checked')){
        role = 'admin';
    }
    
    $.ajax({ 
        url: url +'/users/' + $('#up_un').val(),
        type: 'PUT',
        data : {
        'firstname'    : $('#up_fn').val(),
        'lastname'     : $('#up_ln').val(),
        'username'      : $('#up_un').val(),
        'password'      : $('#up_pw').val(),
        'email'         : $('#up_em').val(),
        'age'           : $('#up_ag').val(),
        'role'          : role},
        success: function(){
            location.reload();
        }
    });
    
    $('#update_modal').modal('toggle');

    
});

$('#rem_user').click( function() 
{
    $.ajax({
        url: url + '/users/' + $('#rem_un').val(),
        type: 'DELETE',
        success: function(){
            location.reload();
        }
    });

    $('#rem_modal').modal('toggle');
});


$('#search_user').click( function() 
{
    $.get( url +'/users/' + $('#srch_un').val(),
    function(data, status)
    {   
        var user = JSON.parse(data);
        
        if(user == undefined) {
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
        console.log(status);
    });

    $('#search_modal').modal('hide');
    $('#update_modal').modal('show');
      
});
