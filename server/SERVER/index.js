var express = require("express");
const { Socket } = require("socket.io");
var app = express();
app.use(express.static("public"));
app.set("view engine", "ejs");
app.set("views", "./views");

var server = require("http").Server(app);
var io = require("socket.io")(server);
server.listen(3000);

io.on("connection", function(socket){
    ////////////////
    console.log(socket.id + " connected")
    socket.on('disconnect', function () {
        console.log(socket.id + " disconnected")
    })

    socket.on("button", function (data) {
        if (data == "on") {
            console.log('button on')
            client.publish('buttona', 'on');
        }
        else {
            console.log('button off')
            client.publish('buttona', 'off');
        }
    })

    socket.on("button2", function (data) {
        if (data == "on") {
            console.log('button2 on')
            client.publish('buttonb', 'on');
        }
        else {
            console.log('button2 off')
            client.publish('buttonb', 'off');
        }
    })
    ////////////
});


app.get("/", function(req, res){
    res.render("mcb3.ejs");
});
app.get('/history', function (req, res) {
    res.render('history.ejs')
});

const mqtt = require('mqtt');
// var client = mqtt.connect('mqtt:/172.20.10.2');
// var client = mqtt.connect('mqtt:/192.168.1.64');

var client = mqtt.connect('mqtt:/192.168.43.105');

// var client = mqtt.connect('mqtt:/192.168.0.121');

/////////////
var express = require('express');
var app = express();
// var bodyParser = require('body-parser');
var mysql = require('mysql');

var dbConn = mysql.createConnection({
    host: 'localhost',
    user: 'root',
    password: '',
    database: 'wsn2' 
});
dbConn.connect();

dbConn.query('CREATE TABLE IF NOT EXISTS data_sensors2(id int(10) AUTO_INCREMENT,temp int(10),humid int(10),light int(10),dust int(10),created_at timestamp DEFAULT CURRENT_TIMESTAMP,PRIMARY KEY(id))')
dbConn.query('SELECT * FROM `data_sensors2` ORDER BY created_at DESC LIMIT 1', function (error, results) {
    if (error) throw error;
    console.log({results});
});
//////////////

client.on('connect', function () {
    client.subscribe('dataSensor');
    console.log('Client has subcribed successfully');

});
// var cnt_check = 0;
client.on('message', function (topic, message,packet) {
    ////////////////////////////
    let tmp = message.toString().split(',');
    var temp = tmp[0];
    var humid = tmp[1];
    var light = tmp[2];
    var dust = tmp[3];

    ////////////////////////////////////
    console.log("===================");
    console.log(temp);
    console.log(humid);
    console.log(light);
    console.log(dust);
    console.log("===================");
    ///////////////////////////////////
    var sql = "insert into data_sensors2(temp,humid,light,dust) value ( '"+temp+"' , "+humid+" , "+light+" ,"+dust+")"
    var show = 'SELECT * FROM `data_sensors2` ORDER BY created_at DESC LIMIT 1'
    dbConn.query(sql,function(err,result){
        if (err) throw err
    })
    
    dbConn.query(show,function(err,res){
        if(err) throw err
        console.log(res)
    })
    //////////////////send data to mcb3
    var sql1 = "SELECT * FROM data_sensors2 ORDER BY ID DESC limit 1"
    dbConn.query(sql1, function (err, result, fields) {
        if (err) throw err;
        console.log("Data selected");
        result.forEach(function (value) {
            var m_time = value.created_at.toString().slice(4, 24);
            // var newTemp = value.temp
            // var newHumi = value.humid
            // var newLight = value.light
            // var newDust = value.dust
            /////
            // console.log(m_time + " " + value.temp + " " + value.humid + " " + value.light + " " + value.dust);
            /////
            io.sockets.emit('server-update-data', { id: value.id, time: m_time, temp: value.temp, humid: value.humid, light: value.light, dust: value.dust });

            console.log("okokokokok");
        })
    });
    ////////////////send data to history
    // var sql1 = "SELECT * FROM data_sensors2 ORDER BY ID DESC limit 1"
    dbConn.query(sql1, function (err, result, fields) {
        if (err) throw err;
        console.log("Full Data selected");
        var fullData = []
        result.forEach(function (value) {
            var m_time = value.created_at.toString().slice(4, 24);
            fullData.push({ id: value.id, time: m_time, temp: value.temp, humid: value.humid, light: value.light, dust: value.dust });

            

            console.log("okokokokok2");
        })

        // console.log(fullData);
        io.sockets.emit('send-full', fullData);
    });
})
