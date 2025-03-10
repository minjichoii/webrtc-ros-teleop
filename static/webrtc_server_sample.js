import http from "http"
import express from "express";
import SocketIO from "socket.io";
const app = express();
app.set("view engine", "pug");
app.set("views", __dirname + "/views");
app.use("/public", express.static(__dirname + "/public"));
app.get("/", (_, res) => res.render("home"));
const httpServer = http.createServer(app);
const wsServer = SocketIO(httpServer);
const port = 3000;
const handleListen = () => console.log(`Listening on http://localhost:${port}`)
httpServer.listen(port, handleListen);
wsServer.on("connection", socket => {
    socket.on("join", (roomName, done) => {
        socket.join(roomName);
        done();
        socket.to(roomName).emit("welcome");
    });
    socket.on("offer", (offer, roomName) => {
        socket.to(roomName).emit("offer", offer);
    });
    socket.on("answer", (answer, roomName) => {
        socket.to(roomName).emit("answer", answer);
    });
    socket.on("ice", (ice, roomName) => {
        socket.to(roomName).emit("ice", ice);
    })
});