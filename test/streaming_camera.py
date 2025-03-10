import asyncio
import cv2
import av
import rospy
import websockets
from aiortc import RTCPeerConnection, MediaStreamTrack, RTCSessionDescription
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# WebRTC PeerConnection
pc = RTCPeerConnection()
bridge = CvBridge()

# set video stream track (send video received from ROS to WebRTC )
class VideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.frame = None #save the recent frames

    def set_frame(self, frame):
        self.frame = frame

    async def recv(self):
        # when webrtc request the new frame, give the recent frame
        pts, time_base = await self.next_timestamp()
        # opencv->webrtc
        frame = av.VideoFrame.from_ndarray(self.frame, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
    
# add a video track on WebRTC
video_track = VideoStreamTrack()
pc.addTrack(video_track)

# subscrive the realsense topic in ros and send it to webrtc
def image_callback(msg):
    # camera topic -> frame -> webrtc track
    frame = bridge.imgmsg_to_cv2(msg, "bgr8") # ros image -> opencv
    video_track.set_frame(frame) # update the webrtc video track

# init ros node and subscribe the camera topic
rospy.init_node('webrtc_video_sender')
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

#async def signaling():
#    async with websockets.connect("ws://server:port") as ws: # insert port
#        # create webrtc offer and send signaling server
#        offer = await pc.createOffer()
#        await pc.setLocalDescription(offer)
#        await ws.send(offer.sdp) #send sdp

        # receive the sdp answer from server
#        answer_sdp = await ws.recv()
#        await pc.setRemoteDescription(answer_sdp)

async def signaling(websocket, path):
    # 로컬에서 WebRTC offer와 answer 테스트
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    await websocket.send(offer.sdp)

    answer_sdp = await websocket.recv()
    answer_description = RTCSessionDescription(sdp=answer_sdp, type="answer")
    await pc.setRemoteDescription(answer_description)

start_server = websockets.serve(signaling, "localhost", 5000)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever

