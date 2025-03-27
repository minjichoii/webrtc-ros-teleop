import socketio
import asyncio
import os
import sys

from logging_setup import logger
from config import SOCKET_SERVER_URL

# 상위 디렉토리 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

class SocketClient:
    """Socket.IO 클라이언트"""
    
    def __init__(self, room_id):
        self.sio = socketio.AsyncClient()
        self.room_id = room_id
        self.connected = False
        
        # 이벤트 핸들러 콜백
        self.on_connect_callback = None
        self.on_offer_callback = None
        self.on_answer_callback = None
        
    def setup_events(self):
        """Socket.IO 이벤트 핸들러 설정"""
        
        @self.sio.event
        async def connect():
            logger.info("Socket.IO 서버에 연결됨")
            self.connected = True
            
            logger.info(f"방 {self.room_id} 참가 요청 전송 중...")
            await self.sio.emit("join_room", {"room": self.room_id})
            logger.info(f"방 {self.room_id}에 참가 요청 전송 완료")
            
            if self.on_connect_callback:
                await self.on_connect_callback()

        @self.sio.event
        async def connect_error(error):
            logger.error(f"Socket 연결 오류: {error}")
            self.connected = False

        @self.sio.event
        async def all_users(all_users):
            logger.info(f"👪️ all_users 이벤트: {all_users}")
            if all_users and len(all_users) > 0:
                logger.info("✅ 다른 사용자가 존재함. Offer를 보낼 수 있음.")
                
                if self.on_connect_callback:
                    await self.on_connect_callback()
        
        @self.sio.event
        async def getOffer(sdp):
            logger.info("getOffer 이벤트 수신")
            if self.on_offer_callback:
                await self.on_offer_callback(sdp)

        @self.sio.event
        async def getAnswer(sdp):
            logger.info("🔵 getAnswer 이벤트 수신")
            if self.on_answer_callback:
                await self.on_answer_callback(sdp)
    
    def set_callbacks(self, on_connect=None, on_offer=None, on_answer=None):
        """콜백 함수 설정"""
        self.on_connect_callback = on_connect
        self.on_offer_callback = on_offer
        self.on_answer_callback = on_answer
    
    async def connect(self):
        """Socket.IO 서버에 연결"""
        logger.info(f"Socket.IO 서버 연결 시도: {SOCKET_SERVER_URL}")
        try:
            await self.sio.connect(
                SOCKET_SERVER_URL,
                transports=["websocket", "polling"],
                wait_timeout=10,
            )
        except Exception as e:
            logger.error(f"Socket.IO 연결 오류: {e}")
            raise
    
    async def emit_offer(self, sdp):
        """Offer SDP 전송"""
        if self.connected:
            await self.sio.emit("offer", sdp)
            logger.info("완성된 Offer 전송 완료")
    
    async def emit_answer(self, sdp):
        """Answer SDP 전송"""
        if self.connected:
            await self.sio.emit("answer", sdp)
            logger.info("완성된 Answer 전송 완료")
    
    async def disconnect(self):
        """Socket.IO 연결 종료"""
        if self.sio.connected:
            await self.sio.disconnect()
            self.connected = False