import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROSCameraClient:
    """ROS 카메라 데이터를 처리하는 클래스"""
    
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_frame = None
        self._initialize_ros()
        
    def _initialize_ros(self):
        """ROS 노드 초기화 및 토픽 구독"""
        rospy.init_node("webrtc_camera_node", anonymous=True)
        print("ROS 노드 초기화 완료")
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        
    def image_callback(self, msg):
        """ROS 이미지를 OpenCV 형식으로 변환"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.latest_frame is None:
                print("이미지 변환 완료")
            self.latest_frame = frame
            
        except Exception as e:
            rospy.logerr(f"이미지 변환 실패: {e}")
    
    def get_latest_frame(self):
        """최신 프레임 반환"""
        return self.latest_frame