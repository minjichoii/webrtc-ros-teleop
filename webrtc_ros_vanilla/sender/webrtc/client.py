import asyncio
from logging_setup import logger
from config import DEFAULT_ROOM_ID
from ros.camera import ROSCameraClient
from ros.video_track import ROSVideoStreamTrack
from webrtc.peer_connection import WebRTCPeerConnection
from websocket_client.socket_client import SocketClient

import os
import sys

# 현재 스크립트가 있는 디렉토리와 한 단계 위 디렉토리를 sys.path에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir)  # 현재 디렉토리 추가
sys.path.append(parent_dir)   # 상위 디렉토리 추가

class WebRTCClient:
    """WebRTC 클라이언트"""
    
    def __init__(self, room_id=DEFAULT_ROOM_ID):
        # 방 ID 확인
        self.room_id = room_id
        if not self.room_id:
            raise ValueError("🚨 ROOM_ID 환경 변수가 설정되지 않았습니다! .env 파일을 확인하세요.")
        
        logger.info(f"방 ID 확인: {self.room_id}")
        
        # 컴포넌트 초기화
        self.ros_camera = ROSCameraClient()
        self.peer_connection = WebRTCPeerConnection()
        self.socket_client = SocketClient(self.room_id)
        
        # 데이터 채널 핸들러 설정
        self.peer_connection.setup_data_channel_handler(self.setup_data_channel)
        
        # WebRTC 비디오 트랙 설정
        self.video_track = ROSVideoStreamTrack(self.ros_camera)
        self.peer_connection.add_track(self.video_track)
        
        # Socket.IO 이벤트 핸들러 설정
        self.socket_client.set_callbacks(
            on_connect=self.on_socket_connect,
            on_offer=self.on_offer,
            on_answer=self.on_answer
        )
        self.socket_client.setup_events()
        
        # 연결 상태
        self.connected = False
    
    async def on_socket_connect(self):
        """Socket.IO 연결 성공 시 호출"""
        # 방 참가 후 짧은 지연 시간 후 무조건 offer 생성 시도
        await asyncio.sleep(3)
        logger.info("방 참가 후 자동 offer 생성 시도")
        await self.create_offer()
    
    async def on_offer(self, sdp):
        """Offer SDP 수신 시 호출"""
        await self.create_answer(sdp)
    
    async def on_answer(self, sdp):
        """Answer SDP 수신 시 호출"""
        success = await self.peer_connection.set_remote_description(sdp)
        if not success:
            # 오류 발생 시 재연결 시도
            logger.info("오류로 인해 연결 재초기화 중...")
            await asyncio.sleep(2)
            self.peer_connection.initialize()
    
    def setup_data_channel(self, channel):
        """데이터 채널 이벤트 핸들러 설정"""
        
        @channel.on("open")
        def on_open():
            logger.info(f"데이터 채널 열림: {channel.label}")
            self.connected = True
        
        @channel.on("close")
        def on_close():
            logger.info(f"데이터 채널 닫힘: {channel.label}")
            self.connected = False

        @channel.on("message")
        def on_message(message):
            logger.info(f"메시지 수신: {message}")
    
    async def create_offer(self):
        """Offer 생성 및 전송"""
        offer_sdp = await self.peer_connection.create_offer()
        if offer_sdp:
            await self.socket_client.emit_offer(offer_sdp)
    
    async def create_answer(self, sdp):
        """Answer 생성 및 전송"""
        answer_sdp = await self.peer_connection.create_answer(sdp)
        if answer_sdp:
            await self.socket_client.emit_answer(answer_sdp)
    
    async def connect(self):
        """서버 연결"""
        await self.socket_client.connect()
    
    async def disconnect(self):
        """연결 종료"""
        # PeerConnection 종료
        await self.peer_connection.close()
        
        # Socket.IO 연결 종료
        await self.socket_client.disconnect()