
if(!!window.EventSource){

    var source = new EventSource('/events');

    source.addEventListener('new_reading', function(e){
        console.log("new_reading", e.data);
        var dataObject = JSON.parse(e.data);
        console.log(dataObject);
        document.getElementById('tempDisplay').innerHTML = dataObject.temperature;
        document.getElementById('humDisplay').innerHTML = dataObject.humidity;
        document.getElementById('motStatDisplay').innerHTML = dataObject.motStat;

    }, false);

}