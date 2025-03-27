import os
from dotenv import load_dotenv
from aiortc import RTCConfiguration, RTCIceServer

load_dotenv()

# 서버 URL
SOCKET_SERVER_URL = os.getenv("SOCKET_SERVER_URL", "http://default-url.com")
DEFAULT_ROOM_ID = os.getenv("ROOM_ID", "1234")

# ICE 서버 설정
PC_CONFIG = RTCConfiguration(
    iceServers=[
        RTCIceServer(urls="stun:stun.l.google.com:19302"),
        RTCIceServer(urls="stun:stun1.l.google.com:19302"),
        RTCIceServer(urls="stun:stun2.l.google.com:19302"),
        RTCIceServer(urls="stun:stun3.l.google.com:19302"),
    ]
)