#!/usr/bin/env python3
import rospy
import asyncio
import requests
from aiortc import RTCPeerConnection, RTCSessionDescription

async def send_offer():
    rospy.init_node("webrtc_sender", anonymous=True)

    # 1. WebRTC PeerConnection 생성
    pc = RTCPeerConnection()

    # 2. 데이터 채널 추가
    channel = pc.createDataChannel("ros_channel")

    async def send_message():
        while not rospy.is_shutdown():
            message = "Hello from ROS WebRTC!"
            channel.send(message)
            rospy.loginfo(f"📩 Sent: {message}")
            await asyncio.sleep(2)  # 2초마다 전송

    # 3. 데이터 채널이 열린 후 메시지를 보내도록 설정
    @channel.on("open")
    def on_open():
        rospy.loginfo("✅ Data channel is open! Sending messages...")
        asyncio.create_task(send_message())

    # 4. ICE 후보 생성 시 서버로 전송
    @pc.on("icecandidate")
    def on_ice_candidate(candidate):
        if candidate:
            rospy.loginfo(f"📌 Generated ICE Candidate: {candidate}")
            requests.post("http://192.168.1.104:8080/save_candidate", json={"candidate": candidate.to_sdp()})

    # 5. Offer 생성
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    rospy.loginfo(f"📩 Generated Offer SDP:\n{offer.sdp}")

    # 6. 시그널링 서버로 Offer 전송
    response = requests.post("http://192.168.1.104:8080/save_offer", json={"sdp": offer.sdp, "type": "offer"})
    rospy.loginfo(f"📩 Offer sent to signaling server: {response.json()}")

    # 7. Answer를 기다림
    while True:
        response = requests.get("http://192.168.1.104:8080/get_answer")
        if response.status_code == 200 and "sdp" in response.json():
            answer = response.json()
            await pc.setRemoteDescription(RTCSessionDescription(answer["sdp"], answer["type"]))
            rospy.loginfo("✅ Connected! Waiting for ICE candidates...")
            break
        await asyncio.sleep(1)

    # **8. 강제로 ICE 후보를 생성 (트리거)**
    pc.createDataChannel("trigger_ice")  # ✅ await 제거 (오류 해결)
    await asyncio.sleep(2)  # ICE 후보가 생성될 시간을 줌

if __name__ == "__main__":
    asyncio.run(send_offer())

