
var app = require('express')()
var http = require('http').Server(app)
var io = require('socket.io')(http)
http.listen(4000, function() {
    console.log('server listening : localhost:4000')
});
io.on('connection', function(socket) {
    console.log("new connection")
    socket.on('data', function(data) {
        console.log(data)
    })
    socket.on('disconnect', function(socket) {
        console.log("disconnected")
    })
});