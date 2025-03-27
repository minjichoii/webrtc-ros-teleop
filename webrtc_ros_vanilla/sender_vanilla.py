import asyncio
import json
import os
import platform
import ssl
import time
import logging
import cv2
import rospy
import numpy as np
import socketio
from geometry_msgs.msg import Twist
from pathlib import Path
from aiohttp import web
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aiortc import (
    RTCConfiguration,
    RTCDataChannel,
    RTCIceServer,
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.contrib.media import MediaPlayer, MediaRecorder
from av import VideoFrame
from dotenv import load_dotenv

load_dotenv()

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 서버 URL
SOCKET_SERVER_URL = os.getenv("SOCKET_SERVER_URL", "http://default-url.com")

# ICE 서버 설정
pc_config = RTCConfiguration(
    iceServers=[
        RTCIceServer(urls="stun:stun.l.google.com:19302"),
        RTCIceServer(urls="stun:stun1.l.google.com:19302"),
        RTCIceServer(urls="stun:stun2.l.google.com:19302"),
        RTCIceServer(urls="stun:stun3.l.google.com:19302"),
    ]
)

class WebRTCClient:
    def __init__(self):
        # Socket.IO 클라이언트 초기화
        self.sio = socketio.AsyncClient()
        self.pc = None
        self.data_channel = None
        self.connected = False
        self.room_id = "1234"  # 기본 방 ID

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print("-- ROS cmd_vel 퍼블리셔 설정 완료 --")

        if not self.room_id:
            raise ValueError("🚨 ROOM_ID 환경 변수가 설정되지 않았습니다! .env 파일을 확인하세요.")

        # ROS 카메라 스트림 설정
        self.bridge = CvBridge()
        self.latest_frame = None 
        print("-- ROS 카메라 스트림 설정 완료 --")

        # ROS 카메라 토픽 구독
        rospy.init_node("webrtc_camera_node", anonymous=True)
        print("-- ROS 노드 초기화 완료 --")
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Socket 이벤트 핸들러 설정
        self.setup_socket_events()

    def image_callback(self, msg):
        # ROS 이미지를 OpenCV 형식으로 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.latest_frame is None:
                print("Complete: ROS Image ---> OpenCV form")
            self.latest_frame = frame
            
        except Exception as e:
            rospy.logerr(f"Failed: ROS Imgae ---> OpenCV form : {e}")

    def setup_socket_events(self):
        print(f"방 ID 확인: {self.room_id}")
        
        @self.sio.event
        async def connect():
            print("Socket.IO 서버에 연결됨")
            await self.initialize_connection()
            print(f"방 {self.room_id} 참가 요청 전송 중...")
            await self.sio.emit("join_room", {"room": self.room_id})
            print(f"방 {self.room_id}에 참가 요청 전송 완료")

            # 방 참가 후 짧은 지연 시간 후 무조건 offer 생성 시도
            await asyncio.sleep(3)
            print("방 참가 후 자동 offer 생성 시도")
            await self.create_offer()  # all_users 이벤트와 무관하게 offer 시도

        @self.sio.event
        async def connect_error(error):
            logger.error(f"Socket 연결 오류: {error}")

        @self.sio.event
        async def all_users(all_users):
            print(f"👪️ all_users 이벤트: {all_users}")
            if all_users and len(all_users) > 0:
                print("✅ 다른 사용자가 존재함. Offer를 보낼 수 있음.")

                # Offer 생성 및 전송
                if self.pc and self.pc.localDescription:
                    print("📡 Offer가 이미 생성됨. 서버가 Offer를 전달했는지 확인 중...")
                else:
                    print("⚠️ Offer가 생성되지 않음. 새 Offer 생성 시작...")
                    await self.create_offer()
        
        @self.sio.event
        async def getOffer(sdp):
            print("🔵 getOffer 이벤트 수신")
            await self.create_answer(sdp)

        @self.sio.event
        async def getAnswer(sdp):
            print("🔵 getAnswer 이벤트 수신")
            
            try:
                sdp_type = sdp.get("type", "")
                sdp_sdp = sdp.get("sdp", "")
                # print(f"✅ 수신한 SDP 타입: '{sdp_type}'")

                # RTCSessionDescription 생성
                rtc_sdp = RTCSessionDescription(sdp=sdp_sdp, type=sdp_type)
                print(f"✅ RTCSessionDescription 생성 성공! 타입: {rtc_sdp.type}, SDP 길이: {len(rtc_sdp.sdp)}")
                
                # Remote Description 설정
                await self.pc.setRemoteDescription(rtc_sdp)
                print("✅ Remote Description 설정 완료!")

            except Exception as e:
                print(f"Remote Description 설정 오류: {e}")
                print("오류로 인해 연결 재초기화 중...")
                
                # 오류 발생 시 재연결 시도
                await asyncio.sleep(2)
                await self.initialize_connection()

        # 바닐라 ICE 방식에서는 필요 없지만, 호환성을 위해 유지
        @self.sio.event
        async def getCandidate(candidate):
            print(f"getCandidate 이벤트 수신 (바닐라 ICE에서는 사용되지 않음)")
            print(f"수신된 ICE 후보: {candidate}")
    
    async def initialize_connection(self):
        print("WebRTC 연결 초기화")
        
        # 기존 연결 정리
        if self.pc:
            await self.pc.close()
            print("기존 RTCPeerConnection 종료")

        # 연결 상태 초기화
        self.connected = False

        # RTCPeerConnection 초기화
        self.pc = RTCPeerConnection(configuration=pc_config)
        print("RTCPeerConnection 초기화 완료")

        if self.pc:
            print("✅ 새 RTCPeerConnection 생성 성공")
        else:
            print("⛔️ RTCPeerConnection 생성 실패")
        
        # 바닐라 ICE 방식에서는 개별 ICE 후보 전송 안함
        # ICE 상태 모니터링 (디버깅용)
        @self.pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            print(f"🚦 ICE Gathering 상태 변경: {self.pc.iceGatheringState}")
            
            if self.pc.iceGatheringState == "complete":
                print("✅ ICE Gathering 완료! 모든 ICE 후보가 수집됨.")
                
                # ICE 후보 통계 확인 (디버깅용)
                ice_stats = await self.pc.getStats()
                candidate_count = 0
                
                for stat in ice_stats.values():
                    if stat.type == "local-candidate":
                        candidate_count += 1
                
                print(f"✅ SDP 전송 후 추가 ICE 후보 수집: {candidate_count}개")

        # 비디오 트랙 설정
        video_track = ROSVideoStreamTrack(self)
        if video_track:
            print("✅ ROSVideoStreamTrack 생성 성공")
        else:
            print("❌ ROSVideoStreamTrack 생성 실패")

        # 비디오 트랙 추가
        try:
            self.pc.addTrack(video_track)
            print("✅ 비디오 트랙 추가 완료")
        except Exception as e:
            print(f"❌ 비디오 트랙 추가 실패: {e}")

        # 데이터 채널 생성없이 수신만 하고 있었음. 데이터 채널 생성!
        self.data_channel = self.pc.createDataChannel("textChannel")
        print(f"🔄 데이터 채널 생성: {self.data_channel}")

        self.setup_data_channel(self.data_channel)

        # 데이터 채널 이벤트 핸들러
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            print(f"🔄 데이터 채널 수신: {channel.label}")
            self.setup_data_channel(channel)
        
        # ICE 연결 상태 변화 이벤트 핸들러
        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"ICE 연결 상태 변경: {self.pc.iceConnectionState}")

            if self.pc.iceConnectionState in ["connected", "completed"]:
                self.connected = True
                print("✅ WebRTC 연결 성공!")
            elif self.pc.iceConnectionState == "disconnected":
                self.connected = False
                print("❌ WebRTC 연결 끊김")
                # 5초 후 재연결 시도
                await asyncio.sleep(5)
                await self.initialize_connection()
            elif self.pc.iceConnectionState == "failed":
                self.connected = False
                print("❌ WebRTC 연결 실패")
                # 즉시 재연결 시도
                await self.initialize_connection()
            elif self.pc.iceConnectionState == "closed":
                self.connected = False
                print("❌ WebRTC 연결이 closed 상태로 변경됨")

                try:
                    print(f"  -Signaling 상태: {self.pc.signalingState}")
                    print(f"  - ICE Gathering 상태: {self.pc.iceGatheringState}")

                    stats = await self.pc.getStats()
                    connection_stats = {}
                    for stat in stats.values():
                        if stat.type == "candidate-pair" and stat.state == "failed":
                            print(f"  - 실패한 ICE 후보 쌍 발견: {stat}")
                        elif stat.type == "transport":
                            connection_stats = stat

                    if connection_stats:
                        print(f"  - 연결 통계: {connection_stats}")

                    print(f"  - 소켓 연결 상태: {self.sio.connected}")

                except Exception as e:
                    print(f"  - 연결 진단 중 오류: {e}")

                # 연결 재시도
                print("  - 재연결 시도 중...")
                await asyncio.sleep(1)
                await self.initialize_connection()

    def setup_data_channel(self, channel):
        # print(f"데이터 채널 설정 중: {channel.label}, 상태: {channel.readyState}")

        @channel.on("open")
        def on_open():
            print(f"🔄 데이터 채널 열림: {channel.label}")
            self.connected = True

            # 채널이 열렸을 때 테스트 메시지 전송
            channel.send(json.dumps({"type": "status", "message": "데이터 채널 연결됨"}))
        
        @channel.on("close")
        def on_close():
            print(f"🔄 데이터 채널 닫힘: {channel.label}")
            self.connected = False

        @channel.on("message")
        def on_message(message):
            # print(f"✉️ 메시지 수신: {message}")

            try:
                # JSON 형식인지 확인
                if isinstance(message, str) and message.startswith('{'):
                    data = json.loads(message)

                    if data.get('type') == 'robot_command':
                        linear = data.get('linear', {})
                        angular = data.get('angular', {})

                        # 로봇 이동 명령 출력
                        print(f"✉️ 로봇 이동 명령 수신: linear_x={linear.get('x', 0)}, angular_z={angular.get('z', 0)}")

                        # ROS Twist 메시지 생성
                        twist = Twist()
                        twist.linear.x = float(linear.get('x', 0))
                        twist.linear.y = float(linear.get('y', 0))
                        twist.linear.z = float(linear.get('z', 0))
                        twist.angular.x = float(angular.get('x', 0))
                        twist.angular.y = float(angular.get('y', 0))
                        twist.angular.z = float(angular.get('z', 0))

                        # cmd_vel 토픽으로 퍼블리시
                        self.cmd_vel_pub.publish(twist)
                        print(f"cmd_vel 퍼블리시 완료: {twist}")
                    else:
                        # 일반 JSON 메시지 처리
                        print(f"🤖 로봇 명령 JSON 데이터 수신: {data}")
                else:
                    # 일반 텍스트 메시지 처리
                    print(f"✉️ 일반 텍스트 메시지 수신: {message}")
            except Exception as e:
                print(f"메시지 처리 중 오류 발생: {e}")
    
    # ICE 후보 수집 함수 수정
    async def wait_for_ice_gathering_complete(self):
        print("ICE 후보 수집 시작...")
        while self.pc.iceGatheringState != "complete":
            # 현재 수집된 ICE 후보 확인
            stats = await self.pc.getStats()
            candidates = [s for s in stats.values() if s.type == "local-candidate"]
            print(f"현재 {len(candidates)}개 ICE 후보 수집됨, 상태: {self.pc.iceGatheringState}")
            await asyncio.sleep(0.5)
        print("ICE 후보 수집 완료!")
            
        return self.pc.iceGatheringState == "complete"

    async def create_offer(self):
        """바닐라 ICE 방식으로 Offer 생성 및 전송"""
        print("Offer 생성 시작 (바닐라 ICE 방식)")
        if not self.pc:
            logger.error("PeerConnection이 초기화되지 않았습니다")
            return
        
        try:
            # Offer 생성
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            print("Local Description 설정 완료")
            
            # ICE 후보 수집 완료 대기
            await self.wait_for_ice_gathering_complete()
            
            # 완성된 SDP (ICE 후보 포함) 가져오기
            complete_sdp = self.pc.localDescription
            print(f"완성된 Offer SDP:\n{complete_sdp.sdp}")
            
            # Offer 전송
            await self.sio.emit("offer", {
                "type": complete_sdp.type,
                "sdp": complete_sdp.sdp
            })
            print("완성된 Offer 전송 완료")
            
        except Exception as e:
            logger.error(f"Offer 생성 오류: {e}")

    async def create_answer(self, sdp):
        """바닐라 ICE 방식으로 Answer 생성 및 전송"""
        print("Answer 생성 시작 (바닐라 ICE 방식)")
        if not self.pc:
            print("PeerConnection이 초기화되지 않았습니다")
            return
        
        try:
            # Remote Description 설정
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp["type"], sdp["sdp"]))
            print("Remote Description 설정 완료")
            
            # Answer 생성
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)
            print("Local Description 설정 완료")
            
            # ICE 후보 수집 완료 대기
            await self.wait_for_ice_gathering_complete()
            
            # 완성된 SDP (ICE 후보 포함) 가져오기
            complete_sdp = self.pc.localDescription
            
            # Answer 전송
            await self.sio.emit("answer", {
                "type": complete_sdp.type,
                "sdp": complete_sdp.sdp
            })
            print("완성된 Answer 전송 완료")
            
        except Exception as e:
            logger.error(f"Answer 생성 오류: {e}")

    async def connect(self):
        """Socket.IO 서버에 연결"""
        print(f"Socket.IO 서버 연결 시도: {SOCKET_SERVER_URL}")
        try:
            await self.sio.connect(
                SOCKET_SERVER_URL,
                transports=["websocket", "polling"],
                wait_timeout=10,
            )
        except Exception as e:
            print(f"Socket.IO 연결 오류: {e}")
            raise

    async def disconnect(self):
        """연결 종료"""
        print("연결 종료 중...")
        
        # PeerConnection 종료
        if self.pc:
            await self.pc.close()
            
        # Socket.IO 연결 종료
        if self.sio.connected:
            await self.sio.disconnect()


class ROSVideoStreamTrack(VideoStreamTrack):
    """ROS 카메라 데이터를 WebRTC 비디오 트랙으로 변환"""

    def __init__(self, webrtc_client):
        super().__init__()
        self.webrtc_client = webrtc_client
        self.frame_count = 0
        self.start_time = time.time()
        self.last_log_time = time.time()

    async def recv(self):
        """ROS 카메라에서 최신 프레임을 가져와 WebRTC로 전송"""
        self.frame_count += 1
        
        # 성능 로깅 (5초마다)
        current_time = time.time()
        if current_time - self.last_log_time > 5:
            elapsed = current_time - self.last_log_time
            fps = self.frame_count / elapsed
            print(f"📊 비디오 성능: {self.frame_count}프레임, {fps:.1f} FPS")
            self.frame_count = 0
            self.last_log_time = current_time

        # 최신 프레임 가져오기
        frame = self.webrtc_client.latest_frame
        if frame is None:
            # 프레임이 없으면 검은 화면 생성
            frame = np.zeros((480, 640, 3), np.uint8)
        else:
            # 깊은 복사로 레이스 컨디션 방지
            frame = frame.copy()

        # OpenCV BGR에서 WebRTC RGB로 변환
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 타임스탬프 생성
        pts, time_base = await self.next_timestamp()

        # VideoFrame 생성
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame


async def main():
    # WebRTC 클라이언트 생성
    client = WebRTCClient()

    try:
        # 서버 연결
        await client.connect()
        
        # 연결 유지 및 상태 모니터링
        while True:
            await asyncio.sleep(1)
            
            # 연결 상태 출력
            if client.connected:
                print("상태: 연결됨")
            else:
                print("상태: 연결 안됨")
                
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        # 연결 종료
        await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())