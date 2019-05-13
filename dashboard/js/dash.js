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
                    name = user['first-name'] + ' ' + user['last-name'];
                    console.log(name);
                },
            });
            time = parseInt(element["end"]) - parseInt(element["start"]);
            total_time += time;

            var date = new Date(parseInt(element["end"])*1000);

            var formattedTime = date.getDay() + '/' + date.getMonth() + '/' + date.getFullYear()%2000 + ' - ' + date.getHours() + ':' + date.getMinutes();

            var to_append = '<p><span class=\'font-weight-bold\'>' + formattedTime + '</span> - O utilizador ' + name + ' usou a cadeira ' + 
            element['chairId'] + ' durante ' + parseInt(time/60) + ' minutos</p>'

            $('#history').append(to_append);
        });

        $('#time_nav').text(parseInt(total_time/60) + ' Min');

    });

    $.get('http://localhost:5000/chairs' ,function( c_data )
    {
        var chairs = JSON.parse(c_data);
        console.log(chairs.length);

        $('#active_chairs').text("1 / 2");
    });

    $.get('http://localhost:5000/users' ,function( u_data )
    {
        var users = JSON.parse(u_data);
        console.log(users.length);

        $('#active_users').text("2 / 6");
    });

});