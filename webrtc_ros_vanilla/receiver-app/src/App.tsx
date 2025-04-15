// src/App.tsx
import React, { useEffect, useState, useCallback, useRef } from "react";
import io from "socket.io-client";

// 컴포넌트 임포트 
import MessageInput from "./components/MessageInput";
import AppLayout from "./components/AppLayout";
import ConnectionStatus from "./components/ConnectionStatus";
import VideoPlayer from "./components/VideoPlayer";
import RobotControlPanel from "./components/RobotControlPanel";

// 커스텀 훅 임포트
import useVideoStream from "./hooks/useVideoStream";
import useMessage from "./hooks/useMessage";
import { useRobotControl } from "./hooks/useRobotControl"; 

// 설정 가져오기
import { SOCKET_SERVER_URL, ROOM_ID, PC_CONFIG } from "./config/constants";

const App = () => {
  // 연결 상태 관리
  const [connectionStatus, setConnectionStatus] = useState<string>("연결 중...");
  const [reconnectTrigger, setReconnectTrigger] = useState<number>(0);
  
  // 로봇 상태 관리 
  const [rosVersion, setRosVersion] = useState<string>("ros1");
  const [movementState, setMovementState] = useState<string>('정지');
  const [speedLevel, setSpeedLevel] = useState<number>(5);
  
  // 참조값 관리
  const socketRef = useRef<SocketIOClient.Socket>(null);
  const pcRef = useRef<RTCPeerConnection>(null);
  const dataChannelRef = useRef<RTCDataChannel>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // 데이터 채널 상태 추적
  const [dataChannelReady, setDataChannelReady] = useState<boolean>(false);

  // 비디오 스트림 관리 훅
  const {
    remoteVideoRef,
    isVideoPlaying,
    setIsVideoPlaying,
    playVideo,
  } = useVideoStream();

  // 메시지 입력/전송 관리 훅
  const {
    message,
    setMessage,
    inputFocused,
    setInputFocused,
    handleInputClick,
    handleInputChange,
    handleFocus,
    handleBlur,
    handleKeyPress,
    handleMessageSubmit,
  } = useMessage({
    onSendMessage: (msg) => {
      if (!dataChannelReady || !dataChannelRef.current) {
        alert("연결이 아직 준비되지 않았습니다. 잠시 후 다시 시도해주세요.");
        return;
      }
      
      console.log("메시지 전송:", msg);
      dataChannelRef.current.send(msg);
      
      // 메시지 전송 후 다시 입력창에 포커스
      if (inputRef.current) {
        inputRef.current.focus();
      }
    },
    initialRef: inputRef
  });

  // ICE 수집 완료 대기 함수 - 바닐라 ICE 방식
  const waitForIceGatheringComplete = useCallback((pc: RTCPeerConnection): Promise<void> => {
    console.log("ICE 후보 수집 대기 중...");
    return new Promise<void>((resolve) => {
      // 이미 완료되었으면 즉시 해결
      if (pc.iceGatheringState === "complete") {
        console.log("ICE 후보 수집이 이미 완료됨");
        resolve();
        return;
      }
      
      // 상태 변경 확인 함수
      const checkState = () => {
        if (pc.iceGatheringState === "complete") {
          console.log("ICE 후보 수집 완료");
          pc.removeEventListener("icegatheringstatechange", checkState);
          resolve();
        }
      };
      
      // 이벤트 리스너 등록
      pc.addEventListener("icegatheringstatechange", checkState);
      
      // 타임아웃 설정 (10초)
      setTimeout(() => {
        if (pc.iceGatheringState !== "complete") {
          console.warn("ICE 후보 수집 타임아웃, 현재까지 수집된 후보로 진행");
          pc.removeEventListener("icegatheringstatechange", checkState);
          resolve();
        }
      }, 10000);
    });
  }, []);

  // 속도 레벨에 따른 실제 속도값 계산
  const calculateVelocity = useCallback((value: number): number => {
    if (value === 0) return 0;
    const baseSpeed = 0.2; 
    const maxSpeed = 1.0;
    return baseSpeed + ((maxSpeed - baseSpeed) * (speedLevel - 1) / 9) * value;
  }, [speedLevel]);

  // 이동 방향에 따른 상태 텍스트 반환
  const getMovementState = useCallback((linearX: number, angularZ: number): string => {
    if (linearX === 0 && angularZ === 0) return '정지';
    if (linearX > 0 && angularZ === 0) return '전진';
    if (linearX < 0 && angularZ === 0) return '후진';
    if (linearX === 0 && angularZ > 0) return '좌회전';
    if (linearX === 0 && angularZ < 0) return '우회전';
    if (linearX > 0 && angularZ > 0) return '왼쪽 전진';
    if (linearX > 0 && angularZ < 0) return '오른쪽 전진';
    if (linearX < 0 && angularZ < 0) return '왼쪽 후진';
    if (linearX < 0 && angularZ > 0) return '오른쪽 후진';
    return '알 수 없음';
  }, []);

  // 로봇에 속도 명령 전송
  const sendRobotVelocity = useCallback((linearX: number, angularZ: number, keyInfo?: string) => {
    if (!dataChannelReady || !dataChannelRef.current) {
      console.log('데이터 채널이 연결되어 있지 않습니다.');
      return;
    }

    try {
      // 로봇 제어 메시지 형식 생성
      const robotCommand = {
        type: 'robot_command',
        linear: { x: calculateVelocity(linearX), y: 0, z: 0 },
        angular: { x: 0, y: 0, z: calculateVelocity(angularZ) },
        keyInfo: keyInfo || '알 수 없음'
      };

      // 데이터 채널을 통해 명령 전송
      dataChannelRef.current.send(JSON.stringify(robotCommand));
      console.log('로봇 명령 전송:', robotCommand, '원본값:', { linearX, angularZ, keyInfo });

      // 버튼 두개 한꺼번에 클릭되는 상황 방지-> 지연시킴
      setTimeout(() => {
        setMovementState(getMovementState(linearX, angularZ));
      }, 10);
    } catch (error) {
      console.error('로봇 명령 전송 오류:', error);

      checkDataChannelState();
    }
  }, [calculateVelocity, getMovementState, dataChannelReady]);

  // 데이터 채널 상태 확인 및 업데이트
  const checkDataChannelState = useCallback(() => {
    const isReady = dataChannelRef.current && dataChannelRef.current.readyState === "open";
    console.log("데이터 채널 상태 확인:", isReady ? "준비됨" : "준비되지 않음", dataChannelRef.current?.readyState);
    setDataChannelReady(Boolean(isReady));
    return Boolean(isReady);
  }, []);

  // WebRTC 연결 설정
  const setupConnection = useCallback(async () => {
    console.log("연결 설정 시작");
    try {
      if (!(pcRef.current && socketRef.current)) {
        console.error("PeerConnection 또는 Socket이 초기화되지 않음");
        return;
      }
      
      // 데이터 채널 생성
      dataChannelRef.current = pcRef.current.createDataChannel("textChannel", {
        ordered: true // 순서 보장 옵션 추가
      });
      console.log("데이터 채널 생성 상태:", dataChannelRef.current.readyState);
      console.log("데이터 채널 생성:", dataChannelRef.current.label);
      
      dataChannelRef.current.onopen = () => {
        console.log("데이터 채널이 열렸습니다.");
        setConnectionStatus("연결됨");
        setDataChannelReady(true);

        // 데이터 채널이 열리면 입력창에 포커스
        setTimeout(() => {
          if (inputRef.current) {
            inputRef.current.focus();
          }
        }, 500);
      };

      dataChannelRef.current.onmessage = (event) => {
        console.log("데이터 채널 메시지 수신:", event.data);
      };

      dataChannelRef.current.onerror = (error) => {
        console.error("데이터 채널 오류:", error);
        setDataChannelReady(false);
      };
      
      dataChannelRef.current.onclose = () => {
        console.log("데이터 채널이 닫혔습니다.");
        setConnectionStatus("연결 끊김");
        setDataChannelReady(false);

        setTimeout(() => {
          console.log("데이터 채널 닫힘 감지 - 재연결 시도");

          if (pcRef.current) {
            pcRef.current.close();
            pcRef.current = null;
          }
          setReconnectTrigger(prev => prev + 1);
        }, 1000);
      };
      
      // 바닐라 ICE 방식: ICE 후보를 개별적으로 전송하지 않고 SDP에 포함시킴
      pcRef.current.onicecandidate = (e) => {
        // 바닐라 ICE 방식에서는 이 이벤트에서 아무것도 하지 않음
        // 대신 waitForIceGatheringComplete 함수에서 상태 변화를 감지함
        if (e.candidate) {
          console.log("ICE 후보 생성 (저장됨, 전송하지 않음):", 
                     e.candidate.candidate.substr(0, 50) + "...");
        }
      };
      
      // ICE 연결 상태 변경 감지
      pcRef.current.oniceconnectionstatechange = () => {
        console.log("ICE 연결 상태 변경:", pcRef.current?.iceConnectionState);
        
        if (pcRef.current?.iceConnectionState === "connected") {
          setConnectionStatus("연결됨");
          checkDataChannelState();
        } else if (pcRef.current?.iceConnectionState === "disconnected") {
          setConnectionStatus("연결 끊김");
          setDataChannelReady(false);
          // 5초 후 재연결 시도
          setTimeout(() => {
            setReconnectTrigger(prev => prev + 1);
          }, 5000);
        } else if (pcRef.current?.iceConnectionState === "failed") {
          setConnectionStatus("연결 실패");
          setIsVideoPlaying(false);
          setDataChannelReady(false);
          // 즉시 재연결 시도
          setReconnectTrigger(prev => prev + 1);
        }
      };
      
      // 원격 트랙 수신 처리
      pcRef.current.ontrack = (ev) => {
        console.log("원격 트랙 수신:", ev.streams.length, "스트림");
        if (ev.streams && ev.streams[0]) {
          console.log("원격 스트림 ID:", ev.streams[0].id);
          console.log("원격 비디오 트랙:", ev.streams[0].getVideoTracks().length);
          
          if (remoteVideoRef.current) {
            remoteVideoRef.current.srcObject = ev.streams[0];
            console.log("원격 비디오 요소에 스트림 설정 완료");
            
            // 비디오 로딩 이벤트 추가
            remoteVideoRef.current.onloadedmetadata = () => {
              console.log("비디오 메타데이터 로드됨");
              // 자동 재생 시도하지 않음 - 사용자 상호작용 필요
              setIsVideoPlaying(false);
            };
          }
        } else {
          console.warn("원격 스트림이 없습니다.");
        }
      };

      // 원격 데이터 채널 수신 처리
      pcRef.current.ondatachannel = (event) => {
        console.log("원격 데이터 채널 수신:", event.channel.label);

        // 로봇 측에서 생성한 데이터 채널로 업데이트
        dataChannelRef.current = event.channel;

        // 이벤트 핸들러 재설정
        event.channel.onopen = () => {
          console.log("수신된 데이터 채널 열림");
          setDataChannelReady(true);
          setConnectionStatus("연결됨");
        };

        event.channel.onclose = () => {
          console.log("수신된 데이터 채널 닫힘");
          setDataChannelReady(false);
          setConnectionStatus("연결됨");
        };

        event.channel.onmessage = (msgEvent) => {
          console.log("수신된 데이터 채널 메시지:", msgEvent.data)
        }
      }
      
      // 방 참가 이벤트 발생
      socketRef.current.emit("join_room", {
        room: ROOM_ID,
      });
      console.log("방 참가 요청 전송");
      
    } catch (e) {
      console.error("연결 설정 오류:", e);
    }
  }, [remoteVideoRef, setIsVideoPlaying, checkDataChannelState]);

  // Offer 생성 - 바닐라 ICE 방식
  const createOffer = useCallback(async () => {
    console.log("Offer 생성 시작 (바닐라 ICE 방식)");
    if (!(pcRef.current && socketRef.current)) {
      console.error("PeerConnection 또는 Socket이 초기화되지 않음");
      return;
    }
    
    try {
      // Offer 생성
      const sdp = await pcRef.current.createOffer({
        offerToReceiveAudio: true,
        offerToReceiveVideo: true,
      });
      console.log("Offer SDP 생성 완료");
      
      // 로컬 설명 설정
      await pcRef.current.setLocalDescription(new RTCSessionDescription(sdp));
      console.log("Local description 설정 완료");
      
      // ICE 후보 수집 완료 대기 - 바닐라 ICE 방식
      await waitForIceGatheringComplete(pcRef.current);
      
      // 완성된 SDP 가져오기 (ICE 후보 포함)
      const completeOffer = pcRef.current.localDescription;
      console.log("ICE 후보가 포함된 완성된 Offer 생성 완료");
      
      // 완성된 Offer 전송
      socketRef.current.emit("offer", completeOffer);
      console.log("완성된 Offer 전송 완료");
      
    } catch (e) {
      console.error("Offer 생성 오류:", e);
    }
  }, [waitForIceGatheringComplete]);

  // Answer 생성 - 바닐라 ICE 방식
  const createAnswer = useCallback(async (sdp: RTCSessionDescription) => {
    console.log("Answer 생성 시작 (바닐라 ICE 방식)");
    if (!(pcRef.current && socketRef.current)) {
      console.error("PeerConnection 또는 Socket이 초기화되지 않음");
      return;
    }
    
    try {
      console.log("Remote description 설정 시도 (offer)");
      console.log("현재 signaling 상태:", pcRef.current.signalingState);
      
      // Remote 설명 설정 (ICE 후보 포함된 Offer)
      await pcRef.current.setRemoteDescription(new RTCSessionDescription(sdp));
      console.log("Remote description 설정 완료");
      
      // Answer 생성
      const mySdp = await pcRef.current.createAnswer({
        offerToReceiveVideo: true,
        offerToReceiveAudio: true,
      });
      console.log("Answer SDP 생성 완료");
      
      // 로컬 설명 설정
      await pcRef.current.setLocalDescription(new RTCSessionDescription(mySdp));
      console.log("Local description 설정 완료");
      
      // ICE 후보 수집 완료 대기 - 바닐라 ICE 방식
      await waitForIceGatheringComplete(pcRef.current);
      
      // 완성된 SDP 가져오기 (ICE 후보 포함)
      const completeAnswer = pcRef.current.localDescription;
      console.log("ICE 후보가 포함된 완성된 Answer 생성 완료");
      
      // 완성된 Answer 전송
      socketRef.current.emit("answer", completeAnswer);
      console.log("완성된 Answer 전송 완료");
      
    } catch (e) {
      console.error("Answer 생성 오류:", e);
    }
  }, [waitForIceGatheringComplete]);

  // 연결 초기화
  const initializeConnection = useCallback(() => {
    console.log("연결 초기화");
    
    // 기존 연결 정리
    if (pcRef.current) {
      pcRef.current.close();
    }
    
    // 비디오 재생 상태 초기화
    setIsVideoPlaying(false);
    setDataChannelReady(false);
    
    // RTCPeerConnection 초기화
    pcRef.current = new RTCPeerConnection(PC_CONFIG);
    console.log("RTCPeerConnection 초기화 완료");
    
    // 연결 설정
    setupConnection();
  }, [setupConnection, setIsVideoPlaying]);

  // 원본 로봇 제어 훅 사용 - RobotControlPanel에 전달하지 않고 내부적으로만 사용
  useRobotControl({
    sendRobotVelocity,
    isDataChannelOpen: dataChannelReady,
    rosVersion
  });

  // 초기 설정
  useEffect(() => {
    console.log("컴포넌트 마운트 - 연결 초기화");
    
    // Socket.IO 연결 설정
    const socket = io.connect(SOCKET_SERVER_URL, {
      transports: ['websocket', 'polling'],
      forceNew: true,
      reconnection: true,
      reconnectionAttempts: 5,
      reconnectionDelay: 1000
    });
    socketRef.current = socket;
    
    console.log("Socket.IO 연결 시도:", SOCKET_SERVER_URL);
    
    // Socket.IO 이벤트 리스너 설정
    socket.on("connect", () => {
      console.log("Socket.IO 서버에 연결됨");
      initializeConnection();
    });
    
    socket.on("connect_error", (error: any) => {
      console.error("Socket.IO 연결 오류:", error);
      setConnectionStatus("서버 연결 오류");
    });
    
    socket.on("all_users", (allUsers: Array<{ id: string }>) => {
      console.log("all_users 이벤트 수신:", allUsers);
      if (allUsers.length > 0) {
        createOffer();
      }
    });

    socket.on("getOffer", (sdp: RTCSessionDescription) => {
      console.log("getOffer 이벤트 수신");
      createAnswer(sdp);
    });

    socket.on("getAnswer", async (sdp: RTCSessionDescription) => {
      console.log("getAnswer 이벤트 수신");
      if (!pcRef.current) return;
      
      // 현재 연결 상태 확인
      const currentState = pcRef.current.signalingState;
      console.log("Current signaling state:", currentState);
      
      try {
        // stable 상태가 아닐 때만 setRemoteDescription 실행
        if (currentState !== "stable") {
          await pcRef.current.setRemoteDescription(new RTCSessionDescription(sdp));
          console.log("Remote description 설정 완료 (answer)");

          setTimeout(() => {
            checkDataChannelState();
          }, 1000);
        } else {
          console.log("이미 stable 상태, answer 무시");
        }
      } catch (error) {
        console.error("Remote description 설정 오류:", error);
        // 오류 발생 시 재연결 시도
        setTimeout(() => {
          setReconnectTrigger(prev => prev + 1);
        }, 2000);
      }
    });

    // 바닐라 ICE 방식에서는 개별 ICE 후보 처리가 필요 없지만
    // 호환성을 위해 이벤트 핸들러는 유지
    socket.on("getCandidate", async (candidate: RTCIceCandidateInit) => {
      console.log("getCandidate 이벤트 수신 (바닐라 ICE에서는 일반적으로 무시됨)");
      if (!pcRef.current) return;
      
      // 호환성을 위해 후보 처리는 유지
      try {
        await pcRef.current.addIceCandidate(new RTCIceCandidate(candidate));
        console.log("ICE candidate 추가 완료");
      } catch (error) {
        console.error("ICE candidate 추가 오류:", error);
      }
    });

    // 컴포넌트 마운트 후 입력창에 포커스
    setTimeout(() => {
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }, 1000);

    // 주기적 데이터 채널 상태 확인
    const channelCheckInterval = setInterval(() => {
      checkDataChannelState();
    }, 5000);

    // 컴포넌트 언마운트 시 정리
    return () => {
      console.log("컴포넌트 언마운트 - 연결 정리");
      
      // Socket.IO 연결 종료
      if (socketRef.current) {
        socketRef.current.disconnect();
      }
      
      // RTCPeerConnection 종료
      if (pcRef.current) {
        pcRef.current.close();
      }
    };
  }, [setupConnection, createOffer, createAnswer, initializeConnection, checkDataChannelState]);

  // 재연결 효과
  useEffect(() => {
    if (reconnectTrigger > 0) {
      console.log("재연결 시도:", reconnectTrigger);
      initializeConnection();
    }
  }, [reconnectTrigger, initializeConnection]);

  // 수동 재연결 핸들러
  const handleReconnect = useCallback(() => {
    setReconnectTrigger(prev => prev + 1);
  }, []);

  // 데이터 채널 상태 변경에 대한 효과
  useEffect(() => {
    console.log("데이터 채널 상태 변경:", dataChannelReady ? "준비됨": "준비되지 않음");
  }, [dataChannelReady]);

  return (
    <AppLayout>
      <ConnectionStatus
        connectionStatus={connectionStatus}
        handleReconnect={handleReconnect}
      />

      {/* 좌우 컨텐츠를 담는 컨테이너 */}
      <div style={{
        display: 'flex',
        flexDirection: 'row',
        alignItems: 'flex-start',
        justifyContent: 'center',
        width: '100%',
        gap: '30px',
        flex: 1
      }}>
        {/* 왼쪽 섹션: 비디오와 메시지 입력 */}
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center'
        }}>
          {/* 비디오 섹션 */}
          <VideoPlayer
            remoteVideoRef={remoteVideoRef}
            isVideoPlaying={isVideoPlaying}
            playVideo={playVideo}
          />
          
          {/* 메시지 입력 영역 */}
          <MessageInput
            message={message}
            onChange={handleInputChange}
            onKeyPress={handleKeyPress}
            onFocus={handleFocus}
            onBlur={handleBlur}
            onClick={handleInputClick}
            onSend={handleMessageSubmit}
            inputRef={inputRef}
            isFocused={inputFocused}
          />
        </div>

        {/* 로봇 제어 패널 */}
        <RobotControlPanel
          rosVersion={rosVersion}
          setRosVersion={setRosVersion}
          sendRobotVelocity={sendRobotVelocity}
          movementState={movementState}
          speedLevel={speedLevel}
          setSpeedLevel={setSpeedLevel}
          dataChannelReady={dataChannelReady}
        />
      </div>
    </AppLayout>
  );
};

export default App;