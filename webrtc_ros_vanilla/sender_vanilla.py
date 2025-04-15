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
        self.sio = socketio.AsyncClient(reconnection=True, reconnection_attempts=10, reconnection_delay=1)
        self.pc = None
        self.data_channel = None
        self.connected = False
        self.webrtc_connected = False
        self.data_channel_ready = False
        self.room_id = "1234"  # 기본 방 ID
        self.offer_sent = False
        self.max_reconnect_attempts = 5
        self.reconnect_attempts = 0
        self.reconnect_interval = 2  # 초기 간격 (초)
        self.remote_data_channels = {}  # 원격 데이터 채널 추적

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
        
        # 연결 감시 타이머 설정
        self.connection_monitor_task = None
        self.data_channel_check_task = None

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
            # 연결 시 재연결 시도 카운터 리셋
            self.reconnect_attempts = 0
            self.reconnect_interval = 2
            
            # 연결 상태 모니터링 시작
            if self.connection_monitor_task is None or self.connection_monitor_task.done():
                self.connection_monitor_task = asyncio.create_task(self.monitor_connection())
                
            # 데이터 채널 상태 확인 태스크 시작
            if self.data_channel_check_task is None or self.data_channel_check_task.done():
                self.data_channel_check_task = asyncio.create_task(self.check_data_channel_periodically())
                
            await self.initialize_connection()
            print(f"방 {self.room_id} 참가 요청 전송 중...")
            await self.sio.emit("join_room", {"room": self.room_id})
            print(f"방 {self.room_id}에 참가 요청 전송 완료")

            # 방 참가 후 짧은 지연 시간 후 무조건 offer 생성 시도
            await asyncio.sleep(2)
            print("방 참가 후 자동 offer 생성 시도")
            self.offer_sent = False  # offer 전송 상태 초기화
            await self.create_offer()  # all_users 이벤트와 무관하게 offer 시도

        @self.sio.event
        async def connect_error(error):
            logger.error(f"Socket 연결 오류: {error}")
            self.connected = False
            self.data_channel_ready = False
            await self.try_reconnect()

        @self.sio.event
        async def disconnect():
            print("Socket.IO 서버와 연결이 끊어짐")
            self.connected = False
            self.data_channel_ready = False
            await self.try_reconnect()

        @self.sio.event
        async def all_users(all_users):
            print(f"👪️ all_users 이벤트: {all_users}")
            
            # offer가 아직 전송되지 않았거나 연결이 끊어진 상태라면 offer 생성
            if not self.offer_sent or not self.webrtc_connected:
                print("✅ Offer 생성 시작...")
                await self.create_offer()
            else:
                print("⏭️ Offer가 이미 전송됨. 재전송하지 않음.")

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
                
                # Answer를 받았다면 기본적으로 offer는 성공적으로 전송된 것
                self.offer_sent = True
                
                # 데이터 채널 상태 확인 예약
                asyncio.create_task(self.check_data_channel_after_delay(2))

            except Exception as e:
                print(f"Remote Description 설정 오류: {e}")
                print("오류로 인해 연결 재초기화 중...")
                
                # 오류 발생 시 재연결 시도
                await asyncio.sleep(2)
                self.offer_sent = False  # offer 상태 초기화
                await self.initialize_connection()

        # 바닐라 ICE 방식에서는 필요 없지만, 호환성을 위해 유지
        @self.sio.event
        async def getCandidate(candidate):
            print(f"getCandidate 이벤트 수신 (바닐라 ICE에서는 사용되지 않음)")
            print(f"수신된 ICE 후보: {candidate}")
    
    async def check_data_channel_after_delay(self, delay_seconds):
        """답변을 받은 후 일정 시간 후에 데이터 채널 상태 확인"""
        await asyncio.sleep(delay_seconds)
        self.check_data_channel()
        
    def check_data_channel(self):
        """데이터 채널 상태 확인"""
        if self.data_channel:
            ready = self.data_channel.readyState == "open"
            if ready != self.data_channel_ready:
                self.data_channel_ready = ready
                print(f"데이터 채널 상태 변경: {'준비됨' if ready else '준비되지 않음'}")
            return ready
        return False
        
    async def check_data_channel_periodically(self):
        """주기적으로 데이터 채널 상태 확인"""
        try:
            while True:
                await asyncio.sleep(5)  # 5초마다 확인
                self.check_data_channel()
                
                # 연결되었으나 데이터 채널이 준비되지 않은 경우 재생성 시도
                if self.webrtc_connected and not self.data_channel_ready and self.pc:
                    print("WebRTC 연결은 되었으나 데이터 채널이 준비되지 않음. 데이터 채널 재생성 시도...")
                    
                    # 원래 데이터 채널이 있으면 닫기
                    if self.data_channel:
                        try:
                            self.data_channel.close()
                        except:
                            pass
                    
                    # 새 데이터 채널 생성
                    self.data_channel = self.pc.createDataChannel("textChannel")
                    print(f"새 데이터 채널 생성: {self.data_channel}")
                    self.setup_data_channel(self.data_channel)
        except asyncio.CancelledError:
            print("데이터 채널 확인 태스크 취소됨")
        except Exception as e:
            print(f"데이터 채널 확인 중 오류: {e}")
    
    async def try_reconnect(self):
        """연결이 끊어졌을 때 재연결 시도"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            print(f"최대 재연결 시도 횟수({self.max_reconnect_attempts})에 도달했습니다.")
            return

        self.reconnect_attempts += 1
        wait_time = self.reconnect_interval * self.reconnect_attempts
        print(f"재연결 시도 {self.reconnect_attempts}/{self.max_reconnect_attempts}, {wait_time}초 후 시도...")
        
        await asyncio.sleep(wait_time)
        
        if not self.sio.connected:
            try:
                print("Socket.IO 서버에 재연결 시도...")
                await self.connect()
            except Exception as e:
                print(f"Socket.IO 재연결 실패: {e}")
    
    async def monitor_connection(self):
        """연결 상태를 주기적으로 모니터링하는 비동기 태스크"""
        try:
            while True:
                await asyncio.sleep(5)  # 5초마다 체크
                
                # WebRTC 연결 상태 확인
                if self.pc and self.pc.iceConnectionState not in ["connected", "completed"]:
                    print(f"WebRTC 연결 상태 확인: {self.pc.iceConnectionState}")
                    
                    # 일정 시간 이상 연결되지 않은 경우 offer 재전송
                    if not self.webrtc_connected and not self.offer_sent:
                        print("WebRTC 연결이 없음. Offer 재전송 시도...")
                        await self.create_offer()
                
                # Socket.IO 연결 상태 확인
                if not self.sio.connected:
                    print("Socket.IO 연결이 끊어짐. 재연결 시도...")
                    await self.try_reconnect()
                    
        except asyncio.CancelledError:
            print("연결 모니터링 태스크 취소됨")
        except Exception as e:
            print(f"연결 모니터링 중 오류 발생: {e}")
            
    async def initialize_connection(self):
        print("WebRTC 연결 초기화")
        
        # 기존 연결 정리
        if self.pc:
            await self.pc.close()
            print("기존 RTCPeerConnection 종료")

        # 연결 상태 초기화
        self.connected = False
        self.webrtc_connected = False
        self.data_channel_ready = False
        self.offer_sent = False
        self.remote_data_channels = {}

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

        # 데이터 채널 생성
        self.data_channel = self.pc.createDataChannel("textChannel")
        print(f"🔄 데이터 채널 생성: {self.data_channel}")

        self.setup_data_channel(self.data_channel)

        # 데이터 채널 이벤트 핸들러
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            print(f"🔄 데이터 채널 수신: {channel.label}")
            # 수신된 데이터 채널 추적
            channel_id = channel.label
            self.remote_data_channels[channel_id] = channel
            self.setup_data_channel(channel)
        
        # ICE 연결 상태 변화 이벤트 핸들러
        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"ICE 연결 상태 변경: {self.pc.iceConnectionState}")

            if self.pc.iceConnectionState in ["connected", "completed"]:
                self.connected = True
                self.webrtc_connected = True
                self.reconnect_attempts = 0  # 성공적으로 연결되면 카운터 리셋
                print("✅ WebRTC 연결 성공!")
                
                # 연결 후 데이터 채널 상태 확인
                await asyncio.sleep(1)  # 잠시 대기
                self.check_data_channel()
                
                # 연결 성공 알림 전송
                await self.send_status_message("로봇이 연결되었습니다.")
                
            elif self.pc.iceConnectionState == "disconnected":
                self.connected = False
                self.webrtc_connected = False
                self.data_channel_ready = False
                print("❌ WebRTC 연결 끊김")
                
                # 5초 후 재연결 시도
                await asyncio.sleep(5)
                if self.pc.iceConnectionState != "connected":
                    print("연결이 여전히 끊겨 있음. 재연결 시도...")
                    self.offer_sent = False  # offer 상태 초기화
                    await self.initialize_connection()
                    # 연결 초기화 후 offer 전송
                    await asyncio.sleep(1)
                    await self.create_offer()
            
            elif self.pc.iceConnectionState == "failed":
                self.connected = False
                self.webrtc_connected = False
                self.data_channel_ready = False
                print("❌ WebRTC 연결 실패")
                
                # 즉시 재연결 시도
                self.offer_sent = False  # offer 상태 초기화
                await self.initialize_connection()
                # 연결 초기화 후 offer 전송
                await asyncio.sleep(1)
                await self.create_offer()
            
            elif self.pc.iceConnectionState == "closed":
                self.connected = False
                self.webrtc_connected = False
                self.data_channel_ready = False
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
                self.offer_sent = False  # offer 상태 초기화
                await self.initialize_connection()
                # 연결 초기화 후 offer 전송
                await asyncio.sleep(1)
                await self.create_offer()
                
    async def send_status_message(self, message_text):
        """상태 메시지를 데이터 채널을 통해 전송"""
        # 데이터 채널 상태 확인
        await asyncio.sleep(1)  # 잠시 대기
        self.check_data_channel()
        
        # 자체 데이터 채널 시도
        if self.data_channel and self.data_channel.readyState == "open":
            try:
                self.data_channel.send(json.dumps({
                    "type": "status", 
                    "message": message_text
                }))
                print(f"상태 메시지 전송 성공 (자체 채널): {message_text}")
                return True
            except Exception as e:
                print(f"자체 데이터 채널 전송 실패: {e}")
        
        # 원격 데이터 채널 시도
        for channel_id, channel in self.remote_data_channels.items():
            if channel.readyState == "open":
                try:
                    channel.send(json.dumps({
                        "type": "status", 
                        "message": message_text
                    }))
                    print(f"상태 메시지 전송 성공 (원격 채널 {channel_id}): {message_text}")
                    return True
                except Exception as e:
                    print(f"원격 데이터 채널 전송 실패: {e}")
        
        print(f"상태 메시지 전송 실패 (모든 채널): {message_text}")
        return False

    def setup_data_channel(self, channel):
        # print(f"데이터 채널 설정 중: {channel.label}, 상태: {channel.readyState}")

        @channel.on("open")
        def on_open():
            print(f"🔄 데이터 채널 열림: {channel.label}")
            self.connected = True
            self.data_channel_ready = True

            # 채널이 열렸을 때 테스트 메시지 전송
            try:
                channel.send(json.dumps({"type": "status", "message": "데이터 채널 연결됨"}))
                print(f"테스트 메시지 전송 성공: {channel.label}")
            except Exception as e:
                print(f"테스트 메시지 전송 실패: {e}")
        
        @channel.on("close")
        def on_close():
            print(f"🔄 데이터 채널 닫힘: {channel.label}")
            
            # 해당 채널이 주 데이터 채널인 경우에만 상태 변경
            if channel == self.data_channel:
                self.connected = False
                self.data_channel_ready = False

                # 데이터 채널이 닫히면 새 채널 생성 시도
                asyncio.create_task(self._handle_datachannel_close())
            
            # 원격 채널 목록에서 제거
            if channel.label in self.remote_data_channels:
                del self.remote_data_channels[channel.label]

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
                        
                        # 명령 수신 확인 메시지 회신
                        try:
                            reply = {
                                "type": "command_ack",
                                "status": "success",
                                "command": "robot_command"
                            }
                            channel.send(json.dumps(reply))
                        except Exception as e:
                            print(f"명령 확인 회신 실패: {e}")
                    else:
                        # 일반 JSON 메시지 처리
                        print(f"🤖 로봇 명령 JSON 데이터 수신: {data}")
                else:
                    # 일반 텍스트 메시지 처리
                    print(f"✉️ 일반 텍스트 메시지 수신: {message}")
            except Exception as e:
                print(f"메시지 처리 중 오류 발생: {e}")
    
    # 데이터 채널 닫힘 처리를 위한 메서드
    async def _handle_datachannel_close(self):
        print("데이터 채널 닫힘 처리 중...")
        
        # 연결이 여전히 활성 상태이면 데이터 채널만 재생성
        if self.webrtc_connected and self.pc and self.pc.iceConnectionState in ["connected", "completed"]:
            try:
                print("연결은 유지되어 있음. 데이터 채널만 재생성...")
                self.data_channel = self.pc.createDataChannel("textChannel")
                self.setup_data_channel(self.data_channel)
                print(f"데이터 채널 재생성 완료: {self.data_channel}")
                return
            except Exception as e:
                print(f"데이터 채널 재생성 실패: {e}")
        
        # 연결 자체가 문제인 경우 전체 재연결
        print("전체 연결 재초기화...")
        await self.initialize_connection()
        await asyncio.sleep(1)
        await self.create_offer()
    
    # ICE 후보 수집 함수 수정
    async def wait_for_ice_gathering_complete(self):
        print("ICE 후보 수집 시작...")
        
        # 타임아웃 설정 (최대 10초)
        timeout = 10
        start_time = time.time()
        
        while self.pc.iceGatheringState != "complete":
            # 현재 수집된 ICE 후보 확인
            stats = await self.pc.getStats()
            candidates = [s for s in stats.values() if s.type == "local-candidate"]
            print(f"현재 {len(candidates)}개 ICE 후보 수집됨, 상태: {self.pc.iceGatheringState}")
            
            # 타임아웃 체크
            if time.time() - start_time > timeout:
                print(f"ICE 후보 수집 타임아웃 ({timeout}초). 현재까지 수집된 후보로 진행합니다.")
                break
                
            await asyncio.sleep(0.5)
            
        print("ICE 후보 수집 완료!")
        return True

    async def create_offer(self):
        """바닐라 ICE 방식으로 Offer 생성 및 전송"""
        print("Offer 생성 시작 (바닐라 ICE 방식)")
        if not self.pc:
            logger.error("PeerConnection이 초기화되지 않았습니다")
            return
        
        # 이미 offer를 전송했으면 중복 전송 방지
        if self.offer_sent:
            print("이미 Offer를 전송했습니다. 중복 전송 방지.")
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
            if not complete_sdp:
                print("경고: LocalDescription이 없습니다!")
                return
                
            print(f"완성된 Offer SDP:\n{complete_sdp.sdp}")
            
            # Offer 전송
            await self.sio.emit("offer", {
                "type": complete_sdp.type,
                "sdp": complete_sdp.sdp
            })
            print("완성된 Offer 전송 완료")
            self.offer_sent = True
            
        except Exception as e:
            logger.error(f"Offer 생성 오류: {e}")
            self.offer_sent = False

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
            if not complete_sdp:
                print("경고: LocalDescription이 없습니다!")
                return
            
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
            await self.try_reconnect()

    async def disconnect(self):
        """연결 종료"""
        print("연결 종료 중...")
        
        # 데이터 채널 상태 확인 태스크 취소
        if self.data_channel_check_task:
            self.data_channel_check_task.cancel()
            try:
                await self.data_channel_check_task
            except asyncio.CancelledError:
                pass
        
        # 연결 모니터링 태스크 취소
        if self.connection_monitor_task:
            self.connection_monitor_task.cancel()
            try:
                await self.connection_monitor_task
            except asyncio.CancelledError:
                pass
        
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
            connection_status = "연결됨" if client.webrtc_connected else "연결 안됨"
            data_channel_status = "준비됨" if client.data_channel_ready else "준비 안됨"
            print(f"상태: {connection_status}, 데이터 채널: {data_channel_status}")
                
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        # 연결 종료
        await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())