function get_weather() {
  var python = require("python-shell")
  var path = require("path")

  var city = document.getElementById("city").value
  document.getElementById("city").value = "";

  var options = {
    scriptPath : path.join(__dirname, '/../engine/gtt/')
  }

  var weather = new python('', options);

  weather.on('message', function(message) {
    swal(message);
  })
}
