import asyncio
import json
import os
import platform
import ssl
import time
import logging
import cv2
import rospy
import numpy as np
import socketio
from pathlib import Path
from aiohttp import web
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aiortc import (
    RTCConfiguration,
    RTCDataChannel,
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.contrib.media import MediaPlayer, MediaRecorder
from av import VideoFrame

# set logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# server URL
SOCKET_SERVER_URL = "http://43.201.110.253:8081"

# ICE server
pc_config = RTCConfiguration(
    iceServers=[
        {"urls": ["stun:stun.l.google.com:19302"]}
    ]
)

class WebRTCClient:
    def __init__(self):
        # init the socket client
        self.sio = socketio.AsyncClient()
        self.pc = None
        self.data_channel = None
        self.ice_candidates = []
        self.connected = False
        self.room_id = "1234" # basic room ID

        # set ROS camera stream
        self.bridge = CvBridge()
        self.latest_frame = None 

        # subscribe the ROS camera topic
        rospy.init_node("webrtc_camera_node", anonymous= True)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # socket event handler
        self.setup_socket_events()

    def image_callback(self, msg):
        # convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame =  frame
        except Exception as e:
            rospy.logerr(f"failed to convert image: {e}")

    def setup_socket_events(self):
        @self.sio.event
        async def connect():
            logger.info("Connected to socket server")
            await self.initialize_connection()
            await self.sio.emit("join_room", {"room": self.room_id})
            logger.info(f"방 {self.room_id}에 참가 요청 전송")

        @self.sio.event
        async def connect_error(error):
            logger.error(f"error on connecting socket: {error}")

        @self.sio.event
        async def all_users(all_users):
            logger.info(f"all_users event: {all_users}")
            if all_users and len(all_users)>0:
                await self.create_offer()

        @self.sio.event
        async def getOffer(sdp):
            logger.info("getOffer event")
            await self.create_answer(sdp)

        @self.sio.event
        async def getAnswer(sdp):
            logger.info("getAnswer event")
            if not self.pc:
                return
            
            try:
                await self.pc.setRemoteDescription(RTCSessionDescription(sdp["type"], sdp["sdp"]))
                logger.info("Complete setting Remote description (answer)")
            except Exception as e:
                logger.error(f"Error setting Remote description: {e}")
                # when error occur, try reconnecting
                await asyncio.sleep(2)
                await self.initialize_connection()

        @self.sio.event
        async def getCandidate(candidate):
            if not self.pc:
                return

            try:
                candidate_obj = RTCIceCandidate(
                    component=candidate.get("component", None),
                    foundation=candidate.get("foundation", None),
                    ip=candidate.get("ip", None),
                    port=candidate.get("port", None),
                    priority=candidate.get("priority", None),
                    protocol=candidate.get("protocol", None),
                    type=candidate.get("type", None),
                    sdpMid=candidate.get("sdpMid", None),
                    sdpMLineIndex=candidate.get("sdpMLineIndex", None),
                )
                await self.pc.addIceCandidate(candidate_obj)
                logger.info("ICE candidate added")
            except Exception as e:
                logger.error(f"ICE candidate add error: {e}")
    
    async def initialize_connection(self):
        logger.info("initialize connection")
        
        # organize the before connection
        if self.pc:
            await self.pc.close()

        # initialize the ICE candidate
        self.ice_candidates= []
        self.connected = False

        # initialize the RTCPeerConnection
        self.pc = RTCPeerConnection(configuration=pc_config)
        logger.info("Complete initialize the RTCPeerConnection")

        video_track = ROSVideoStreamTrack(self)
        self.pc.addTrack(video_track)
        
        # create the data channel
        #self.setup_data_channel(self.data_channel)

        # set the media stream
        #await self.setup_media_stream()

        # ICE candidate event handler
        @self.pc.on("icecandidate")
        async def on_ice_candidate(candidate):
            if candidate:
                logger.info(f"create ICE candidate: {candidate.candidate} ")

                # save the ICE candidate
                self.ice_candidates.append(candidate)

                # send ICE candidate
                candidate_dict ={
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                }
                await self.sio.emit("candidate", candidate_dict)

        # data channel event handler
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            logger.info(f"data channel: {channel.label}")
            self.setup_data_channel(channel)

        # changing the state ICE connection event handler
        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logger.info(f"change the state of ICE connection: {self.pc.iceConnectionState}")

            if self.pc.iceConnectionState in ["connected", "completed"]:
                self.connected=True
                logger.info("connected")
            elif self.pc.iceConnectionState == "disconnected":
                self.connected = False
                logger.info("not connected")
                # try to reconnect after 5sec
                await asyncio.sleep(5)
                await self.initialize_connection()
            elif self.pc.iceConnectionState == "failed":
                self.connected = False
                logger.info("failed to connect")
                # try to reconnect at that time
                await self.initialize_connection()


    def setup_data_channel(self, channel):
        @channel.on("open")
        def on_open():
            logger.info(f"opened data channel: {channel.label}")
            self.connected = True
        
        @channel.on("close")
        def on_close():
            logger.info(f"closed data channel: {channel.label}")
            self.connected = False

        @channel.on("message")
        def on_message(message):
            logger.info(f"received message: {message}")
    
    #async def setup_media_stream(self):
    #    logger.info("start to set media stream")

        # create video track and adding
        #video_track = VideoStreamTrack(self)
        #self.pc.addTrack(video_track)
    #    logger.info("complete adding video track")

    async def create_offer(self):
        logger.info("start to create Offer")
        if not self.pc:
            logger.error("didn't initialize PeerConnection")
            return
        
        try:
            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            logger.info("complete to set Local description")

            # send Offer
            await self.sio.emit("offer", {
                "type": offer.type,
                "sdp": offer.sdp
            })
            logger.info("complete sending Offer")
        except Exception as e:
            logger.error(f"error on Offer creation: {e}")

    async def create_answer(self, sdp):
        logger.info("start to create the Answer")
        if not self.pc:
            logger.info("did't initialize the PeerConnection")
            return
        
        try:
            # remote 
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp["type"], sdp["sdp"]))
            logger.info("complete setting Remote descriptuon")

            # create answer
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)
            logger.info("complete setting Local description")

            # send Answer
            await self.sio.emit("answer", {
                "type": answer.type,
                "sdp": answer.sdp
            })
            logger.info("sent answer")
        except Exception as e:
            logger.error(f"error Answer creation: {e}")

    async def connect(self):
        logger.info("trying to connect server")
        await self.sio.connect(SOCKET_SERVER_URL, transports=["websocket", "polling"])

    async def disconnect(self):
        logger.info("finished connection")

        # turn off camera!!! add code!!

        # finish PeerConnection
        if self.pc:
            await self.pc.close()

        # finish the socket connection
        if self.sio.connected:
            await self.sio.disconnect()


class ROSVideoStreamTrack(VideoStreamTrack):
    """ ROS 카메라 데이터를 WebRTC 비디오 트랙으로 변환 """

    def __init__(self, webrtc_client):
        super().__init__()
        self.webrtc_client = webrtc_client
        self.frame_count = 0

    async def recv(self):
        """ ROS 카메라에서 최신 프레임을 가져와 WebRTC로 전송 """
        self.frame_count += 1

        # 최신 프레임 가져오기
        frame = self.webrtc_client.latest_frame
        if frame is None:
            frame = np.zeros((480, 640, 3), np.uint8)  # 프레임이 없으면 검은 화면

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # OpenCV는 BGR, WebRTC는 RGB

        pts, time_base = await self.next_timestamp()

        # WebRTC VideoFrame 생성
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame

async def main():
    # WebRTC client
    client = WebRTCClient()

    try:
        # connect server
        await client.connect()

        # maintain the connection
        if client.connected:
            logger.info("state : connected")
        else:
            looger.info("state: not connected")

    except KeyboardInterrupt:
        logger.info("finished the program")
    finally:
        # finish
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())



