import os
import sys
from aiortc import RTCPeerConnection, RTCSessionDescription
from logging_setup import logger
from config import PC_CONFIG
from webrtc.ice_handler import ICEHandler

# 상위 디렉토리 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

class WebRTCPeerConnection:
    """WebRTC 피어 연결 관리"""
    
    def __init__(self):
        self.pc = None
        self.ice_handler = None
        self.connected = False
        self.initialize()
        
    def initialize(self):
        """RTCPeerConnection 초기화"""
        logger.info("WebRTC 연결 초기화")
        
        # 기존 연결 정리
        if self.pc:
            self.pc = None
            self.pc.close()
            logger.info("기존 RTCPeerConnection 종료")

        # 연결 상태 초기화
        self.connected = False

        # RTCPeerConnection 초기화
        self.pc = RTCPeerConnection(configuration=PC_CONFIG)
        
        if self.pc:
            logger.info("✅ 새 RTCPeerConnection 생성 성공")
        else:
            logger.error("⛔️ RTCPeerConnection 생성 실패")
            
        # ICE 핸들러 초기화
        self.ice_handler = ICEHandler(self.pc)
        self.ice_handler.setup_ice_handlers(
            on_connected=self._on_connected,
            on_disconnected=self._on_disconnected
        )
    
    async def _on_connected(self):
        """연결 성공 시 호출"""
        self.connected = True
    
    async def _on_disconnected(self):
        """연결 실패 시 호출"""
        self.connected = False
        self.pc = None
        self.initialize()
    
    def add_track(self, track):
        """미디어 트랙 추가"""
        return self.pc.addTrack(track)
    
    def setup_data_channel_handler(self, on_datachannel):
        """데이터 채널 이벤트 핸들러 설정"""
        @self.pc.on("datachannel")
        def _on_datachannel(channel):
            logger.info(f"데이터 채널 수신: {channel.label}")
            on_datachannel(channel)
    
    async def create_offer(self):
        """Offer 생성"""
        logger.info("Offer 생성 시작 (바닐라 ICE 방식)")
        if not self.pc:
            logger.error("PeerConnection이 초기화되지 않았습니다")
            return None
        
        try:
            # Offer 생성
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            logger.info("Local Description 설정 완료")
            
            # ICE 후보 수집 완료 대기
            await self.ice_handler.wait_for_ice_gathering_complete()
            
            # 완성된 SDP (ICE 후보 포함) 가져오기
            complete_sdp = self.pc.localDescription
            logger.info(f"완성된 Offer SDP 길이: {len(complete_sdp.sdp)}")
            
            return {
                "type": complete_sdp.type,
                "sdp": complete_sdp.sdp
            }
            
        except Exception as e:
            logger.error(f"Offer 생성 오류: {e}")
            return None
    
    async def create_answer(self, sdp):
        """Answer 생성"""
        logger.info("Answer 생성 시작 (바닐라 ICE 방식)")
        if not self.pc:
            logger.error("PeerConnection이 초기화되지 않았습니다")
            return None
        
        try:
            # Remote Description 설정
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp["type"], sdp["sdp"]))
            logger.info("Remote Description 설정 완료")
            
            # Answer 생성
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)
            logger.info("Local Description 설정 완료")
            
            # ICE 후보 수집 완료 대기
            await self.ice_handler.wait_for_ice_gathering_complete()
            
            # 완성된 SDP (ICE 후보 포함) 가져오기
            complete_sdp = self.pc.localDescription
            
            return {
                "type": complete_sdp.type,
                "sdp": complete_sdp.sdp
            }
            
        except Exception as e:
            logger.error(f"Answer 생성 오류: {e}")
            return None
    
    async def set_remote_description(self, sdp):
        """원격 설명 설정"""
        try:
            # RTCSessionDescription 생성
            rtc_sdp = RTCSessionDescription(sdp=sdp["sdp"], type=sdp["type"])
            logger.info(f"✅ RTCSessionDescription 생성 성공! 타입: {rtc_sdp.type}")
            
            # Remote Description 설정
            await self.pc.setRemoteDescription(rtc_sdp)
            logger.info("✅ Remote Description 설정 완료!")
            return True
            
        except Exception as e:
            logger.error(f"Remote Description 설정 오류: {e}")
            return False
    
    async def close(self):
        """연결 종료"""
        if self.pc:
            await self.pc.close()
            self.pc = None
            self.connected = False