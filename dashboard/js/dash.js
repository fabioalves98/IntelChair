// JS for main dashboard
$(document).ready(function() 
{
    var total_time = 0;

    $.get('http://localhost:5000/history', function( h_data ) 
    {
        var history = JSON.parse(h_data);
        var name, time;
        history.forEach(element => 
        {
            $.ajax( 
            {
                url : 'http://localhost:5000/users/' + element['username'],
                type : 'GET',
                async : false,
                success : function (data) 
                {
                    var user = JSON.parse(data);
                    name = user[0]['firstname'] + ' ' + user[0]['lastname'];
                    console.log(name);
                },
            });
            time = parseInt(element["endTime"]) - parseInt(element["startTime"]);
            total_time += time;

            var date = new Date(parseInt(element["endTime"]));

            var formattedTime = date.getDate() + '/' + (date.getMonth()+1) + '/' + date.getFullYear()%2000 + ' - ' + date.getHours() + ':' + date.getMinutes();

            var to_append = '<p><span class=\'font-weight-bold\'>' + formattedTime + '</span> - O utilizador ' + name + ' usou a cadeira ' + 
            element['chairId'] + ' durante ' + parseInt((time/1000)/60) + ' minutos</p>'

            $('#history').append(to_append);
        });

        $('#time_nav').text(parseInt(total_time/1000/60) + ' Min');

    });

    $.get('http://localhost:5000/chairs' ,function( c_data )
    {
        var chairs = JSON.parse(c_data);
        var active_chairs = 0;
        for(let chair of chairs){
            if(chair.status != "Offline" && chair.status != null){
                active_chairs++;
            }
        }
        $('#active_chairs').text(active_chairs + " / " + chairs.length);
    });

    $.get('http://localhost:5000/users' ,function( u_data )
    {
        var users = JSON.parse(u_data);
        console.log(users.length);

        $('#active_users').text("calc this / " + users.length);
    });

});

function delete_history() {
    $.post( 'http://localhost:5000/remove/history',
    function(data, status)
    {
        console.log(status);
    });

    location.reload();
}