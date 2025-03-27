import asyncio
import os
import sys
from logging_setup import logger

# 상위 디렉토리 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

class ICEHandler:
    """ICE 후보 수집 및 처리"""
    
    def __init__(self, peer_connection):
        self.pc = peer_connection
        
    async def wait_for_ice_gathering_complete(self):
        """ICE 후보 수집 완료 대기"""
        logger.info("ICE 후보 수집 시작...")
        
        while self.pc.iceGatheringState != "complete":
            # 현재 수집된 ICE 후보 확인
            stats = await self.pc.getStats()
            candidates = [s for s in stats.values() if s.type == "local-candidate"]
            logger.info(f"현재 {len(candidates)}개 ICE 후보 수집됨, 상태: {self.pc.iceGatheringState}")
            await asyncio.sleep(0.5)
            
        logger.info("ICE 후보 수집 완료!")
        return True
    
    def setup_ice_handlers(self, on_connected=None, on_disconnected=None):
        """ICE 이벤트 핸들러 설정"""
        
        @self.pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            logger.info(f"🚦 ICE Gathering 상태 변경: {self.pc.iceGatheringState}")
            
            if self.pc.iceGatheringState == "complete":
                logger.info("✅ ICE Gathering 완료! 모든 ICE 후보가 수집됨.")
                
                # ICE 후보 통계 확인 (디버깅용)
                ice_stats = await self.pc.getStats()
                candidate_count = 0
                
                for stat in ice_stats.values():
                    if stat.type == "local-candidate":
                        candidate_count += 1
                
                logger.info(f"✅ SDP 전송 후 추가 ICE 후보 수집: {candidate_count}개")
        
        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logger.info(f"ICE 연결 상태 변경: {self.pc.iceConnectionState}")

            if self.pc.iceConnectionState in ["connected", "completed"]:
                logger.info("✅ WebRTC 연결 성공!")
                if on_connected:
                    await on_connected()
                    
            elif self.pc.iceConnectionState == "disconnected":
                logger.info("❌ WebRTC 연결 끊김")
                if on_disconnected:
                    await on_disconnected()
                    
            elif self.pc.iceConnectionState == "failed":
                logger.info("❌ WebRTC 연결 실패")
                if on_disconnected:
                    await on_disconnected()
                    
            elif self.pc.iceConnectionState == "closed":
                logger.info("❌ WebRTC 연결이 closed 상태로 변경됨")
                if on_disconnected:
                    await on_disconnected()