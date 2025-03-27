import asyncio
from logging_setup import logger
from webrtc.client import WebRTCClient
from config import DEFAULT_ROOM_ID

import os
import sys

# 현재 스크립트가 있는 디렉토리와 한 단계 위 디렉토리를 sys.path에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir)  # 현재 디렉토리 추가
sys.path.append(parent_dir)   # 상위 디렉토리 추가


async def main():
    # WebRTC 클라이언트 생성
    client = WebRTCClient(room_id=DEFAULT_ROOM_ID)

    try:
        # 서버 연결
        await client.connect()
        
        # 연결 유지 및 상태 모니터링
        while True:
            await asyncio.sleep(1)
            
            # 연결 상태 출력
            if client.connected:
                logger.info("상태: 연결됨")
            else:
                logger.info("상태: 연결 안됨")
                
    except KeyboardInterrupt:
        logger.info("프로그램 종료")
    finally:
        # 연결 종료
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())