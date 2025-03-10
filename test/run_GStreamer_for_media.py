#!/usr/bin/env python3

import rospy
import socketio
import subprocess

SIGNALING_SERVER_URL = "http://localhost:5000"
STUN_SERVER = "stun://stun.l.google.com:19302"

sio = socketio.Client()

@sio.event
def connect():
    rospy.loginfo("🔥 Connected to WebRTC Signaling Server")
    sio.emit("join", {"room": "0000"})  # 방 참가

    # ✅ SDP Offer 전송
    send_sdp_offer()

@sio.on("offer")
def on_offer(data):
    rospy.loginfo(f"📩 Received SDP Offer:\n{data['sdp']}\n")

    # ✅ SDP Answer 생성 후 전송
    answer_sdp = generate_sdp_answer()
    sio.emit("answer", {"sdp": answer_sdp, "room": "0000"})

@sio.on("answer")
def on_answer(data):
    rospy.loginfo(f"✅ Received SDP Answer:\n{data['sdp']}\n")

@sio.on("ice-candidate")
def on_ice_candidate(data):
    rospy.loginfo(f"❄️ Received ICE Candidate: {data}")

def send_sdp_offer():
    offer_sdp = "v=0\n..."  # 여기에 실제 SDP Offer 생성 필요
    rospy.loginfo(f"📤 Sending SDP Offer:\n{offer_sdp}\n")
    sio.emit("offer", {"sdp": offer_sdp, "room": "0000"})

def generate_sdp_answer():
    return "v=0\n..."  # 여기에 실제 SDP Answer 생성 필요

def start_gstreamer():
    gst_command = (
        f"gst-launch-1.0 -v webrtcbin name=sendrecv stun-server={STUN_SERVER} "
        "v4l2src device=/dev/video4 ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! "
        "application/x-rtp, encoding-name=VP8, payload=96, clock-rate=90000 ! "
        "rtpjitterbuffer ! queue ! sendrecv.sink_0"
    )

    rospy.loginfo("🎥 Starting WebRTC Video Streaming...")
    process = subprocess.Popen(gst_command, shell=True)
    rospy.spin()
    process.terminate()


if __name__ == "__main__":
    rospy.init_node("gstreamer_webrtc_client")

    # ✅ WebSocket (Socket.IO) 연결
    sio.connect(SIGNALING_SERVER_URL, transports=['polling'])

    # ✅ GStreamer 시작
    start_gstreamer()

    # ✅ WebSocket 유지
    sio.wait()

