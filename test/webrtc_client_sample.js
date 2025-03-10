const socket = io();
const roomName = "room1";
let peerConnection;
let dataChannel;
document.querySelector("form").addEventListener("submit", (event) => {
    event.preventDefault();
    if(dataChannel) {
        const value = document.querySelector("form input").value
        dataChannel.send(value);
    }
});
function onReady() {
    peerConnection = new RTCPeerConnection({
        iceServers: [{ urls: [
            "stun:stun.l.google.com:19302",
            "stun:stun1.l.google.com:19302",
            "stun:stun2.l.google.com:19302",
            "stun:stun3.l.google.com:19302",
            "stun:stun4.l.google.com:19302",
        ]
    }]});
    peerConnection.addEventListener("icecandidate", (event) => {
        socket.emit("ice", event.candidate, roomName);
    });
}
window.onload = () => {
    socket.emit("join", roomName, onReady);
}
function onMessage(msg) {
    const ul = document.querySelector("ul");
    const li = document.createElement("li");
    li.innerText = msg;
    ul.append(li);
}
socket.on("welcome", async () => {
    dataChannel = peerConnection.createDataChannel("chat");
    dataChannel.addEventListener("message", (event) => {
        onMessage(event.data);
    });
    const offer = await peerConnection.createOffer();
    peerConnection.setLocalDescription(offer);
    socket.emit("offer", offer, roomName);
});
socket.on("offer", async (offer) => { 
    peerConnection.addEventListener("datachannel", (event) => {
        dataChannel = event.channel;
        dataChannel.addEventListener("message", (event)=> {
            onMessage(event.data);
        });
    })
    peerConnection.setRemoteDescription(offer);
    const answer = await peerConnection.createAnswer();
    peerConnection.setLocalDescription(answer);
    socket.emit("answer", answer, roomName);
});
socket.on("answer", (answer) => {
    peerConnection.setRemoteDescription(answer);
});
socket.on("ice", (ice) => {
    peerConnection.addIceCandidate(ice);
});