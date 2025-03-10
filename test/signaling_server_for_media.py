#!/usr/bin/env python3

from flask import Flask, request, session
from flask_socketio import SocketIO, emit, join_room
import subprocess
import platform

app = Flask(__name__)
app.config['SECRET_KEY'] = "wubba lubba dub dub"

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading", transports=['polling'])

users_in_room = {}
rooms_sid = {}
names_sid = {}

@socketio.on("connect")
def on_connect():
    sid = request.sid
    print(f"✅ New socket connected: {sid}")

@socketio.on("join")
def on_join(data):
    sid = request.sid
    room_id = data.get("room", "0000")
    join_room(room_id)
    print(f"🚀 Client {sid} joined room: {room_id}")
    emit("room-joined", {"room": room_id}, room=room_id)

@socketio.on("message")
def handle_message(data):
    print(f"📩 Received message: {data}")
    emit("message", f"Echo: {data}", broadcast=True)

@socketio.on("offer")
def handle_offer(data):
    sid = request.sid
    room_id = data.get("room", "0000")
    offer_sdp = data.get("sdp", "")

    print(f"📩 Received SDP Offer from {sid}:\n{offer_sdp}\n")

    # ✅ SDP에서 RTP 포트(5004) 설정
    modified_sdp = modify_sdp_with_rtp_port(offer_sdp, 5004)
    print(f"📩 Modified SDP Offer:\n{modified_sdp}\n")

    emit("offer", {"sdp": modified_sdp, "room": room_id}, room=room_id)
    
    start_gstreamer_receiver()

@socketio.on("answer")
def handle_answer(data):
    sid = request.sid
    room_id = data.get("room", "0000")
    answer_sdp = data.get("sdp", "")

    print(f"✅ Received SDP Answer from {sid}:\n{answer_sdp}\n")

    # ✅ SDP에서 RTP 포트(5004) 설정
    modified_sdp = modify_sdp_with_rtp_port(answer_sdp, 5004)
    print(f"✅ Modified SDP Answer:\n{modified_sdp}\n")

    emit("answer", {"sdp": modified_sdp, "room": room_id}, room=room_id)

@socketio.on("ice-candidate")
def handle_ice_candidate(data):
    print(f"❄️ Received ICE Candidate: {data}")
    emit("ice-candidate", data, broadcast=True)

@socketio.on("disconnect")
def on_disconnect():
    sid = request.sid
    print(f"❌ Client {sid} disconnected")

def start_gstreamer_receiver():
    gst_command = (
        "gst-launch-1.0 -v webrtcbin name=recvbin stun-server=stun://stun.l.google.com:19302 "
        "udpsrc port=5004 caps='application/x-rtp, encoding-name=VP8, payload=96, clock-rate=90000' ! "
        "rtpjitterbuffer latency=500 ! rtpvp8depay ! vp8dec ! videoconvert ! autovideosink"
    )
    print("🎥 Starting GStreamer WebRTC Receiver...")
    subprocess.Popen(gst_command, shell=True)


# ✅ SDP에서 RTP 포트(5004)를 설정하는 함수
def modify_sdp_with_rtp_port(sdp, rtp_port):
    modified_sdp = sdp.replace("a=sendrecv", f"a=sendrecv\na=rtpmap:96 VP8/90000\na=rtcp:{rtp_port}")
    print(f"✅ Modified SDP with RTP port {rtp_port}: {modified_sdp}")
    return modified_sdp

if __name__ == "__main__":
    import rospy
    rospy.init_node("flask_signaling_server", anonymous=True)

    print("🔥 Flask WebRTC Signaling Server Running on http://localhost:5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)

