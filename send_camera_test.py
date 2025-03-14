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
    RTCIceServer,
    RTCIceCandidate,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from aiortc.contrib.media import MediaPlayer, MediaRecorder
from av import VideoFrame
from dotenv import load_dotenv

load_dotenv()

# set logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# server URL
SOCKET_SERVER_URL = os.getenv("SOCKET_SERVER_URL", "http://default-url.com")

# ICE server
pc_config = RTCConfiguration(
    iceServers=[
        RTCIceServer(urls="stun:stun.l.google.com:19302"),
        RTCIceServer(urls="stun:stun1.l.google.com:19302"),
        RTCIceServer(urls="stun:stun2.l.google.com:19302"),
        RTCIceServer(urls="stun:stun3.l.google.com:19302"),
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
        self.room_id = "1234"# basic room ID

        if not self.room_id:
            raise ValueError("🚨 ROOM_ID 환경 변수가 설정되지 않았습니다! .env 파일을 확인하세요.")

        # set ROS camera stream
        self.bridge = CvBridge()
        self.latest_frame = None 
        print("set ROS camera stream")

        # subscribe the ROS camera topic
        rospy.init_node("webrtc_camera_node", anonymous= True)
        print("init ros node")
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # socket event handler
        self.setup_socket_events()

    def image_callback(self, msg):
        # convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.latest_frame is None:
                print("complete to convert image") #check the subscribe
            self.latest_frame =  frame
            
        except Exception as e:
            rospy.logerr(f"failed to convert image: {e}")

    def setup_socket_events(self):
        print(f" ROOM_ID 확인: {self.room_id}") # printed 
        @self.sio.event
        async def connect():
            print(f"Connected to socket server") #complete connection
            await self.initialize_connection() # here
            print(f"방 {self.room_id} 참가 요청 전송 중...")
            await self.sio.emit("join_room", {"room": self.room_id})
            # await self.sio.emit("join_room", json.dumps({"room": self.room_id}))
            print(f"방 {self.room_id}에 참가 요청 전송")

        @self.sio.event
        async def connect_error(error):
            logger.error(f"error on connecting socket: {error}")

        @self.sio.event
        async def all_users(all_users):
            print(f"👪️ all_users event: {all_users}")
            if all_users and len(all_users)>0:
                print("✅ 다른 사용자가 존재함. Offer를 보낼 수 있음.")

                # Offer 전송 여부 확인
                if self.pc and self.pc.localDescription:
                    print("📡 Offer가 이미 생성됨. 서버가 Offer를 전달했는지 확인 중...")
                else:
                    print("⚠️ Offer가 생성되지 않았거나 서버에서 전달되지 않았을 가능성이 있음.")

                if not self.pc.localDescription:
                    await self.create_offer()
        
        #@self.sio.event
        #async def offer_acknowledged():
        #    print("server acknowledged offer reception")

        @self.sio.event
        async def getOffer(sdp):
            print("getOffer event")
            await self.create_answer(sdp)

        @self.sio.event
        async def getAnswer(sdp):
            print("🔵 getAnswer event received")
            # sdp type: answer

            try:
                # ✅ `sdp` 값 확인

                sdp_type = sdp.get("type", "")
                sdp_sdp = sdp.get("sdp", "")
                print(f"✅ Received SDP Type (processed): '{sdp_type}'")

                # 🔹 `RTCSessionDescription` 생성 테스트
                rtc_sdp = RTCSessionDescription(sdp=sdp_sdp, type=sdp_type)
                print(f"✅ Successfully created RTCSessionDescription! Type: {rtc_sdp.type}, SDP Length: {len(rtc_sdp.sdp)}")
                
                # ✅ `RTCSessionDescription`이 정상적으로 생성되었는지 확인 후 `setRemoteDescription()` 실행
                await self.pc.setRemoteDescription(rtc_sdp)
                print("✅ Successfully set Remote Description!")

            except Exception as e:
                print(f"Error setting Remote description: {e}")
                print("re-initializing connection due to error..")
                
                # when error occur, try reconnecting
                await asyncio.sleep(2)
                await self.initialize_connection()

        @self.sio.event
        async def getCandidate(candidate):
            if not self.pc:
                print("No PeerConnection. failed to add ICE")
                return

            try:
                print (f"Received ICE Candidate: {candidate}")
                if "candidate" not in candidate or "sdpMid" not in candidate or "sdpMLineIndex" not in candidate:
                    raise ValueError(f"Invalid ICE candidate data received: {candidate}")
                
                #candidate_obj = RTCIceCandidate(
                #    component=candidate.get("component", None),
                #    foundation=candidate.get("foundation", None),
                #    ip=candidate.get("ip", None),
                #    port=candidate.get("port", None),
                #    priority=candidate.get("priority", None),
                #    protocol=candidate.get("protocol", None),
                #    type=candidate.get("type", None),
                #    sdpMid=candidate.get("sdpMid", None),
                #    sdpMLineIndex=candidate.get("sdpMLineIndex", None),
                #)
                await self.pc.addIceCandidate(candidate)
                print("ICE candidate added")
                
            except Exception as e:
                logger.error(f"ICE candidate add error: {e}")
    
    async def initialize_connection(self):
        print("initialize connection") #printed 
        
        # organize the before connection
        if self.pc:
            await self.pc.close()
            print("completed init RTCPeerConnection")

        # initialize the ICE candidate
        self.ice_candidates= []
        self.connected = False

        # initialize the RTCPeerConnection
        self.pc = RTCPeerConnection(configuration=pc_config)
        print("Complete initialize the RTCPeerConnection")

        if self.pc:
            print("✅ success to create new RTCPeerConnection")
        else:
            print("⛔️ failed to create RTCPeerconnection")
        

        # print(f"connected transiver list: {self.pc.getTransceiver()}")
        
        # create the data channel
        #self.setup_data_channel(self.data_channel)

        # set the media stream
        #await self.setup_media_stream()
        print("ICE candidate handler trying....")
        # ICE candidate event handler
        @self.pc.on("icecandidate")
        async def on_ice_candidate(candidate):
            print("ICE candidate event occur")
            if candidate:
                print(f"create ICE candidate: {candidate.candidate} ")

                # save the ICE candidate
                self.ice_candidates.append(candidate)

                room_id = self.room_id
                # send ICE candidate
                candidate_dict ={
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                }
                print(f"sent ICE candidate")
                await self.sio.emit("candidate", candidate_dict, room=room_id)

        @self.pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            print(f"🚦 ICE Gathering 상태 변경됨: {self.pc.iceGatheringState}")

            if self.pc.iceGatheringState == "complete":
                print("✅ ICE Gathering 완료! 모든 ICE 후보가 수집됨.")

                # ICE 후보 직접 확인
                ice_candidates = await self.pc.getStats()
                found_candidate = False
                candidate_count = 0

                print("🔍 수집된 로컬 ICE 후보 목록:")
                for stat in ice_candidates.values():
                    if stat.type == "local-candidate":
                        print(f"🧊 ICE 후보: {stat}")
                        found_candidate = True
                        candidate_count += 1

                if not found_candidate:
                    print("❌ [ERROR] ICE 후보를 찾지 못함! ")

                    # 🔹 1️⃣ 강제로 ICE Gatherer 실행
                    await self.pc.__gather() 
                    # 🔹 2️⃣ ICE 후보가 없으면 restartIce() 실행
                    if not self.ice_candidates:
                        print("❌ Still no ICE candidates, restarting ICE...")
                        await self.pc.restartIce()

                    # 🔹 3️⃣ ICE Gathering이 완료되지 않았다면 강제 실행
                    if self.pc.iceGatheringState != "complete":
                        print("🔄 ICE Gathering incomplete, forcing execution...")
                        await self.pc.__gather()

                else:
                    print(f"✅ 총 {candidate_count}개의 ICE 후보가 정상적으로 수집됨.")


        video_track = ROSVideoStreamTrack(self) # check here
        if video_track:
            print("success to create ROSVideoStream")
        else:
            print("failed to create ROSVideoStream")

        # self.pc.addTrack(video_track) #transiver 
        try:
            self.pc.addTrack(video_track)
            print("✅ added video track")
        except Exception as e:
            print(f"❌ failed to add video track: {e}")

        # data channel event handler
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            print(f"data channel: {channel.label}")
            self.setup_data_channel(channel)

        
        # changing the state ICE connection event handler
        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"change the state of ICE connection: {self.pc.iceConnectionState}")

            if self.pc.iceConnectionState in ["connected", "completed"]:
                self.connected=True
                print("connected")
            elif self.pc.iceConnectionState == "disconnected":
                self.connected = False
                print("not connected")
                # try to reconnect after 5sec
                await asyncio.sleep(5)
                await self.initialize_connection()
            elif self.pc.iceConnectionState == "failed":
                self.connected = False
                print("failed to connect")
                # try to reconnect at that time
                await self.initialize_connection()


    def setup_data_channel(self, channel):
        @channel.on("open")
        def on_open():
            print(f"opened data channel: {channel.label}")
            self.connected = True
        
        @channel.on("close")
        def on_close():
            print(f"closed data channel: {channel.label}")
            self.connected = False

        @channel.on("message")
        def on_message(message):
            print(f"received message: {message}")
    
    #async def setup_media_stream(self):
    #    print("start to set media stream")

        # create video track and adding
        #video_track = VideoStreamTrack(self)
        #self.pc.addTrack(video_track)
    #    print("complete adding video track")

    async def wait_for_ice_gathering_complete(self):
        while self.pc.iceGatheringState != "complete":
            print(f"current state: {self.pc.iceGatheringState} ")
            await asyncio.sleep(0.5)
        print("complete to gathering ICE")

    async def create_offer(self):
        print("start to create Offer")
        if not self.pc:
            logger.error("didn't initialize PeerConnection")
            return
        
        try:
            offer = await self.pc.createOffer()

            await self.pc.setLocalDescription(offer)

            print("complete to set Local description")
            print(f"offer SDP:\n{offer.sdp}")

            # send Offer
            await self.sio.emit("offer", {
                "type": offer.type,
                "sdp": offer.sdp
            })
            print("complete sending Offer")

            print("checking all users after sendig offer...")
            await self.sio.emit("join_room", {"room": self.room_id})

        except Exception as e:
            logger.error(f"error on Offer creation: {e}")

    async def create_answer(self, sdp):
        print("start to create the Answer")
        if not self.pc:
            print("did't initialize the PeerConnection")
            return
        
        try:
            # remote 
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp["type"], sdp["sdp"]))
            print("complete setting Remote descriptuon")

            # create answer
            answer = await self.pc.createAnswer()
            await self.pc.setLocalDescription(answer)
            print("complete setting Local description")

            # send Answer
            await self.sio.emit("answer", {
                "type": answer.type,
                "sdp": answer.sdp
            })
            print("sent answer")
        except Exception as e:
            logger.error(f"error Answer creation: {e}")

    async def connect(self):
        print("trying to connect server")
        await self.sio.connect(SOCKET_SERVER_URL, transports=["websocket", "polling"])

    async def disconnect(self):
        print("finished connection")

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
        print(f"recv called")
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

        while True:
            await asyncio.sleep(1)

            # maintain the connection
            if client.connected:
                print("state : connected")
            else:
                print("state: not connected")

    except KeyboardInterrupt:
        print("finished the program")
    finally:
        # finish
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())


