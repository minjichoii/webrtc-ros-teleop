#!/usr/bin/env python3
import rospy
import socketio
from std_msgs.msg import String

# WebRTC 시그널링 서버 URL (Socket.IO 기반)
SIGNALING_SERVER_URL = "http://localhost:5000"

sio = socketio.Client()

def connect_polling():
    try:
        sio.connect(SIGNALING_SERVER_URL, transports=['polling'])  # ✅ Polling 강제 사용
        rospy.loginfo("🔥 Polling (Socket.IO) connected")
    except Exception as e:
        rospy.logerr(f"❌ Polling connection error: {e}")

if __name__ == "__main__":
    rospy.init_node("polling_test_node")

    # ✅ Polling 연결
    connect_polling()

    # ✅ 메시지 전송 테스트
    rospy.sleep(1)  # 서버 연결 안정화를 위해 대기
    sio.emit("join", {"room": "0000"})  # Join 이벤트 전송
    rospy.loginfo("📤 Sent JOIN message")

    rospy.sleep(1)
    sio.emit("message", "Test message from polling client!")  # 메시지 전송
    rospy.loginfo("📤 Sent test message")

    # ✅ Polling 연결 유지 (WebSocket처럼 지속적인 연결을 유지해야 함)
    sio.wait()

