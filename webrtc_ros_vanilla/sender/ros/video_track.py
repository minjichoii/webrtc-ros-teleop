import time
import cv2
import numpy as np
from aiortc import VideoStreamTrack
from av import VideoFrame

class ROSVideoStreamTrack(VideoStreamTrack):
    """ROS 카메라 데이터를 WebRTC 비디오 트랙으로 변환"""

    def __init__(self, ros_camera):
        super().__init__()
        self.ros_camera = ros_camera
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
        frame = self.ros_camera.get_latest_frame()
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