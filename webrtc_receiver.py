#!/usr/bin/env python3
import rospy
import asyncio
import requests
from aiortc import RTCPeerConnection, RTCSessionDescription

async def receive_offer():
    rospy.init_node("webrtc_receiver", anonymous=True)

    pc = RTCPeerConnection()

    # 🔥 **ICE 후보를 수신**
    @pc.on("icecandidate")
    def on_ice_candidate(candidate):
        if candidate:
            rospy.loginfo(f"📌 Received ICE Candidate: {candidate}")
            requests.post("http://192.168.1.104:8080/save_candidate", json={"candidate": candidate.to_sdp()})

    while True:
        response = requests.get("http://192.168.1.104:8080/get_offer")
        if response.status_code == 200 and "sdp" in response.json():
            offer = response.json()
            await pc.setRemoteDescription(RTCSessionDescription(offer["sdp"], offer["type"]))
            rospy.loginfo(f"✅ Received Offer: {offer}")
            break
        await asyncio.sleep(1)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    requests.post("http://192.168.1.104:8080/save_answer", json={"sdp": answer.sdp, "type": "answer"})
    rospy.loginfo("✅ Answer sent! Waiting for ICE candidates...")

    # 🔥 **ICE 후보를 가져와 추가**
    while True:
        response = requests.get("http://192.168.1.104:8080/get_candidate")
        if response.status_code == 200 and "candidates" in response.json():
            for candidate in response.json()["candidates"]:
                await pc.addIceCandidate(candidate)
                rospy.loginfo(f"📩 Added ICE Candidate: {candidate}")
            break
        await asyncio.sleep(1)

    rospy.spin()

if __name__ == "__main__":
    asyncio.run(receive_offer())

