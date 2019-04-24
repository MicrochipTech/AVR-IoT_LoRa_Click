var path = require('path');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var ttn = require("ttn")

//var Chart = require('chart.js');


var appID = "mhcptestappttn"
var accessKey = "ttn-account-v2.8JjxCed1rGvYUtCT5a91IO4lfv9DCbSqqdIApbn5q54"


app.use('/static', express.static(path.join(__dirname, 'node_modules')))

app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

http.listen(3000, function(){
  console.log('listening on *:3000');
});

ttn.data(appID, accessKey)
  .then(function (client) {
    client.on("uplink", function (devID, payload) {
      console.log(payload);
      if(payload.is_retry != true) {
        io.emit('uplink message', JSON.stringify(payload.payload_raw));}
    })
  })
  .catch(function (error) {
    console.error("Error", error)
    process.exit(1)
  })
