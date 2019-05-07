// JS for main dashboard
$(document).ready(function() 
{
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

            var date = new Date(parseInt(element["end"])*1000);

            var formattedTime = date.getDay() + '/' + date.getMonth() + '/' + date.getFullYear()%2000 + ' - ' + date.getHours() + ':' + date.getMinutes();

            var to_append = '<p><span class=\'font-weight-bold\'>' + formattedTime + '</span> - O utilizador ' + name + ' usou a cadeira ' + 
            element['chairId'] + ' durante ' + parseInt(time/60) + ' minutos</p>'

            $('#history').append(to_append);
        });

    });

});