import cv2
import asyncio
import json
import rospy
import websockets
import numpy as np
from aiortc import RTCPeerConnection, MediaStreamTrack, RTCSessionDescription, RTCRtpCodecCapability
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from av import VideoFrame

rospy.init_node("ros_webrtc_stream", anonymous=True)

class RealSenseVideoStream(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.frame = None
        self.subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

    async def recv(self):
        if self.frame is None:
            return VideoFrame.from_ndarray(np.zeros((720, 1280, 3), dtype=np.uint8), format="bgr24")

        return VideoFrame.from_ndarray(self.frame, format="bgr24")

async def connect_to_signaling_server():
    uri = "ws://localhost:9000"

    try:
        async with websockets.connect(uri) as websocket:
            print("✅ WebSocket 서버에 연결 성공!")

            pc = RTCPeerConnection()
            video_track = RealSenseVideoStream()

            async for message in websocket:
                try:
                    data = json.loads(message)

                    if data["type"] == "offer":
                        print("✅ Received Offer, checking SDP...")

                        offer_sdp = data["sdp"]
                        print(f"🔍 Offer SDP 확인:\n{offer_sdp}")

                        # ✅ Offer를 설정하기 전에 트랜시버를 추가
                        if not any(t.kind == "video" for t in pc.getTransceivers()):
                            print("🚨 트랜시버가 자동 추가되지 않음! 강제 추가 실행")
                            pc.addTransceiver("video", direction="sendrecv")

                        # ✅ VP8 코덱 설정
                        supported_codecs = [RTCRtpCodecCapability(mimeType="video/VP8", clockRate=90000)]
                        transceiver = pc.getTransceivers()[0]
                        transceiver.setCodecPreferences(supported_codecs)

                        print("✅ VP8 코덱을 사용하도록 설정되었습니다.")
                        print("✅ 트랜시버 추가 완료:", transceiver)

                        # ✅ `addTrack()` 중복 실행 방지
                        if not any(sender.track == video_track for sender in pc.getSenders()):
                            print("✅ 새로운 Sender 추가")
                            sender = pc.addTrack(video_track)
                        else:
                            print("🚨 이미 존재하는 Sender를 중복 추가하려고 함. 무시합니다.")

                        print("✅ 트랜시버에 비디오 트랙 연결 완료")

                        # ✅ Offer 설정
                        await pc.setRemoteDescription(RTCSessionDescription(**data))
                        print("✅ setRemoteDescription 완료!")

                        # ✅ Answer 생성
                        print("✅ Creating Answer...")
                        answer = await pc.createAnswer()

                        print(f"✅ 최종 Answer SDP:\n{answer.sdp}")

                        # ✅ Local Description 설정
                        await pc.setLocalDescription(answer)

                        print("✅ Sending Answer to signaling server...")
                        await websocket.send(json.dumps({"type": "answer", "sdp": answer.sdp}))

                    elif data["type"] == "candidate":
                        await pc.addIceCandidate(data["candidate"])

                except Exception as e:
                    print(f"🚨 내부 메시지 처리 중 오류 발생: {e}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"🚨 WebSocket 연결이 예기치 않게 종료됨: {e}")
    except Exception as e:
        print(f"🚨 WebSocket 연결 오류: {e}")

asyncio.run(connect_to_signaling_server())

