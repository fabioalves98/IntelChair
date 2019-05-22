// JS for main dashboard

$(document).ready(function() 
{
    var total_time = 0;

    $.get(url+'/history', function( h_data ) 
    {
        var history = JSON.parse(h_data);
        var name, time;
        history.forEach(element => 
        {
            $.ajax( 
            {
                url : url + '/users/' + element['username'],
                type : 'GET',
                async : false,
                success : function (data) 
                {
                    var user = JSON.parse(data);
                    name = user['firstname'] + ' ' + user['lastname'];
                },
            });
            time = parseInt(element["endTime"]) - parseInt(element["startTime"]) + 60;
            total_time += time;

            var date = new Date(parseInt(element["endTime"]));

            var formattedTime = date.getDate() + '/' + (date.getMonth()+1) + '/' + date.getFullYear()%2000 + ' - ' + date.getHours() + ':' + date.getMinutes();
            var to_append = '<p><span class=\'font-weight-bold\'>' + formattedTime + '</span> - O utilizador ' + name + ' usou a cadeira ' + 
            element['chair'] + ' durante ' + parseInt((time)/60) + ' minutos</p>'

            $('#history').append(to_append);
        });

        $('#time_nav').text(parseInt(total_time/60) + ' Min');
        
    });

    $.get(url+'/chairs' ,function( c_data )
    {
        var chairs = JSON.parse(c_data);
        var active_chairs = 0;
        for(let chair of chairs)
        {
            if(chair.status != "Offline" && chair.status != null)
            {
                active_chairs++;
            }
        }
        $('#active_chairs').text(active_chairs + " / " + chairs.length);
    });

    $.get(url+'/users' ,function( u_data )
    {
        var users = JSON.parse(u_data);
        var active_users = 0;
        for(let user of users)
        {
            if(user.status != 'Offline')
            {
                active_users++;
            }
        }
        $('#active_users').text(active_users + " / " + users.length);
    });


    $.get(url+'/maps', function(m_data)
    {
        var maps = JSON.parse(m_data) ;
        $('#active_maps').text(maps.length);
    });
});

$('#clear_history').click(function () 
{
    $.ajax(
    {
        url 	: 	url + '/history',
        type 	: 	'DELETE',
        success : function(data)
        {
            location.reload();
        }
    });
});