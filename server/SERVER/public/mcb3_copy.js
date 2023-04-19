let socket = io('http://localhost:3000');

// import { redrawChart } from "./bieu-do.js";



//alert("111");
socket.on("send-full", function (data) {
   //alert("helloooo131231232o");
  // History page
  // $("#id-content").html("")
  // $("#time-content").html("")
  // $("#temp-content").html("")
  // $("#humi-content").html("")
  // $("#dust-content").html("")
  console.log(data)
  data.forEach(function (item) {
    // alert('hellooooooo');
    // $("#id-content").append("<div class='h-para'>" + item.id + "</div>")
    // $("#time-content").append("<div class='h-para'>" + item.time + "</div>")
    // $("#temp-content").append("<div class='h-para'>" + item.temp + "</div>")
    // $("#humi-content").append("<div class='h-para'>" + item.humid + "</div>")
    // $("#light-content").append("<div class='h-para'>" + item.light + "</div>")
    // $("#dust-content").append("<div class='h-para'>" + item.dust + "</div>")

    $("#id-content").prepend("<div class='h-para'>" + item.id + "</div>")
    $("#time-content").prepend("<div class='h-para'>" + item.time + "</div>")
    $("#temp-content").prepend("<div class='h-para'>" + item.temp + "</div>")
    $("#humi-content").prepend("<div class='h-para'>" + item.humid + "</div>")
    $("#light-content").prepend("<div class='h-para'>" + item.light + "</div>")
    $("#dust-content").prepend("<div class='h-para'>" + item.dust + "</div>")

  })
})