const pc = new RTCPeerConnection();
const video = document.getElementById('remoteVideo');

// WebRTC 트랙 수신 처리
pc.ontrack = (event) => {
    // 비디오 스트림을 비디오 태그에 연결
    video.srcObject = event.streams[0];
};


// Python 서버에서 출력된 SDP Offer (위 Python 로그에서 받은 Offer 넣기)
// const offer =''

const socket =new WebSocket('ws://localhost:5000'); //python server address

socket.onmessage = async (message) => {
    const offer_sdp = message.data;
    const offer = new RTCSessionDescription({
        type: 'offer',
        sdp: offer_sdp
    });

    await pc.setRemoteDescription(offer);
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);

    socket.send(answer.sdp);
};

socket.onopen = () => {
    console.log("Succeed connecting WebSocket")
}