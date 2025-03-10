import cv2
import asyncio
import json
import websockets
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaRelay

# WebRTC 연결 초기화
pc = None  # 🔥 기존 PeerConnection을 저장하여 새로고침 시 초기화

relay = MediaRelay()

# 🔥 RealSense 카메라 스트림
class RealSenseStream(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)  # 🔥 RealSense 카메라를 사용할 경우 /dev/videoX 설정 필요

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            return None

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame

# 🔥 WebSocket 연결 및 Offer 생성
async def connect_to_signaling_server():
    global pc  # 🔥 WebRTC 연결 객체를 전역으로 사용

    uri = "ws://localhost:9000"  # 🔥 시그널링 서버 주소

    async with websockets.connect(uri) as websocket:
        print("✅ WebSocket 서버에 연결됨!")

        # 🔥 기존 PeerConnection이 존재하면 초기화
        if pc is not None:
            print("🚨 기존 WebRTC 연결이 존재함! 초기화 실행")
            await pc.close()
            pc = None  # 기존 연결 제거

        # 🔥 새로운 WebRTC PeerConnection 생성
        pc = RTCPeerConnection()
        video_track = RealSenseStream()

        # 🔥 기존에 동일한 트랙이 추가되지 않도록 확인 후 추가
        if not any(sender.track == video_track for sender in pc.getSenders()):
            sender = pc.addTrack(video_track)
            print(f"✅ Sender 추가 완료: {sender}")

        # 🔥 Offer 생성
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # 🔥 Offer 송신
        await websocket.send(json.dumps({
            "type": "offer",
            "sdp": pc.localDescription.sdp
        }))

        # 🔥 응답 수신
        async for message in websocket:
            data = json.loads(message)

            if data["type"] == "answer":
                # ✅ WebRTC가 stable 상태가 아닐 때만 설정
                if pc.signalingState != "stable":
                    await pc.setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type="answer"))
                    print("✅ WebRTC Answer 설정 완료!")
                else:
                    print("🚨 WebRTC가 이미 연결된 상태 (stable). Answer 무시!")

asyncio.run(connect_to_signaling_server())

