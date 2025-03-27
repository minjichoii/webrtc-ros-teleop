import React, { useEffect, useRef, useState, useCallback } from "react";
import io from "socket.io-client";
import TeleopKeypad from "./components/TeleopKeypad";

// process 객체가 없을 경우 폴리필 추가 (타입 오류 해결)
if (typeof window !== 'undefined' && !window.process) {
  window.process = {
    env: {
      NODE_ENV: 'development',
      PUBLIC_URL: '',
      REACT_APP_VERSION: '1.0.0'
    }
  } as any;
}

const pc_config = {
  iceServers: [
    {
      urls: "stun:stun.l.google.com:19302",
    },
  ],
};

const socketServerUrl = process.env.REACT_APP_SOCKET_SERVER_URL;
const roomId = process.env.REACT_APP_ROOM_ID;

console.log("✅ 서버 URL (socketServerUrl):", socketServerUrl);
console.log("✅ 방 ID (roomId):", roomId);


const App = () => {
  {/* 참조값 관리 */}
  // Socket.IO 서버와의 연결 유지
  const socketRef = useRef<SocketIOClient.Socket>(null);
  // RTCPeerConnection 객체 유지 -> WebRTC 연결 관리
  const pcRef = useRef<RTCPeerConnection>(null);
  // <video> DOM 요소에 접근해서 원격 스트림 표시하기 위해 사용
  const remoteVideoRef = useRef<HTMLVideoElement>(null);
  // WebRTC 데이터 채널 유지
  const dataChannelRef = useRef<RTCDataChannel>(null);
  //메시지 입력창 제어
  const inputRef = useRef<HTMLInputElement>(null);
  
  {/* 상태 관리 */}
  // 입력창 메시지 상태 관리
  const [message, setMessage] = useState<string>("");
  // 연결 상태 표시 -> 연결중, 연결됨, 연결 실패
  const [connectionStatus, setConnectionStatus] = useState<string>("연결 중...");
  // 입력창 포커스 상태
  const [inputFocused, setInputFocused] = useState<boolean>(false);
  // 재연결 트리거, 값이 바뀔 때마다 useEffect 통해 다시 연결함
  const [reconnectTrigger, setReconnectTrigger] = useState<number>(0);
  // 비디오 재생 상태
  const [isVideoPlaying, setIsVideoPlaying] = useState<boolean>(false);
  // ROS 버전 
  const [rosVersion, setRosVersion] = useState("ros1"); //기본값이 ros1 
  // 로봇 이동 상태 -> 정지, 전진, 좌회전
  const [movementState, setMovementState] = useState<string>('정지');
  // 로봇 속도 레벨 -> 1~10
  const [speedLevel, setSpeedLevel] = useState<number>(5);

  // ICE 수집 완료 대기 함수 - 바닐라 ICE 방식
  const waitForIceGatheringComplete = (pc: RTCPeerConnection): Promise<void> => {
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
  };

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
      };
      
      dataChannelRef.current.onclose = () => {
        console.log("데이터 채널이 닫혔습니다.");
        setConnectionStatus("연결 끊김");
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
        } else if (pcRef.current?.iceConnectionState === "disconnected") {
          setConnectionStatus("연결 끊김");
          // 5초 후 재연결 시도
          setTimeout(() => {
            setReconnectTrigger(prev => prev + 1);
          }, 5000);
        } else if (pcRef.current?.iceConnectionState === "failed") {
          setConnectionStatus("연결 실패");
          setIsVideoPlaying(false);
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
          // console.log("원격 오디오 트랙:", ev.streams[0].getAudioTracks().length);
          
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
      
      // 방 참가 이벤트 발생
      socketRef.current.emit("join_room", {
        room: roomId,
      });
      console.log("방 참가 요청 전송");
      
    } catch (e) {
      console.error("연결 설정 오류:", e);
    }
  }, []);

  const playVideo = useCallback(() => {
    if (remoteVideoRef.current && remoteVideoRef.current.srcObject && !isVideoPlaying) {
      console.log("비디오 재생 시도");
      
      // 이미 muted 상태인지 확인하고, 필요한 경우에만 변경
      const wasMuted = remoteVideoRef.current.muted;
      if (!wasMuted) {
        remoteVideoRef.current.muted = true;
      }
      
      remoteVideoRef.current.play()
        .then(() => {
          console.log("비디오 재생 성공");
          setIsVideoPlaying(true);
          
          if (!wasMuted) {
            setTimeout(() => {
              if (remoteVideoRef.current) {
                remoteVideoRef.current.muted = false;
              }
            }, 1000);
          }
        })
        .catch(e => {
          console.error("비디오 재생 오류:", e);
          setIsVideoPlaying(false);
        });
    }
  }, [isVideoPlaying]);

  // 속도 레벨에 따른 실제 속도값 계산
  const calculateVelocity = (value: number): number => {
    const baseSpeed = 0.2; 
    const maxSpeed = 1.0;
    return baseSpeed + ((maxSpeed - baseSpeed) * (speedLevel - 1) / 9) * value;
  };

  // 로봇에 속도 명령 전송
  const sendRobotVelocity = useCallback((linearX: number, angularZ: number) => {
    if (!dataChannelRef.current || dataChannelRef.current.readyState !== "open") {
      console.log('데이터 채널이 연결되어 있지 않습니다.');
      return;
    }

    // 로봇 제어 메시지 형식 생성
    const robotCommand = {
      type: 'robot_command',
      linear: { x: calculateVelocity(linearX), y: 0, z: 0 },
      angular: { x: 0, y: 0, z: calculateVelocity(angularZ) }
    };

    // 데이터 채널을 통해 명령 전송
    dataChannelRef.current.send(JSON.stringify(robotCommand));
    console.log('로봇 명령 전송:', robotCommand);
    
    // 이동 상태 설정
    setMovementState(getMovementState(linearX, angularZ));
  }, [dataChannelRef, speedLevel]);

  // 이동 방향에 따른 상태 텍스트 반환
  const getMovementState = (linearX: number, angularZ: number): string => {
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
  };

  // 메시지 전송 함수
  const sendMessage = useCallback(() => {
    if (!dataChannelRef.current || dataChannelRef.current.readyState !== "open") {
      alert("연결이 아직 준비되지 않았습니다. 잠시 후 다시 시도해주세요.");
      return;
    }
    
    if (message.trim() === "") return;
    
    console.log("메시지 전송:", message);
    dataChannelRef.current.send(message);
    setMessage(""); // 입력창 초기화
    
    // 메시지 전송 후 다시 입력창에 포커스
    if (inputRef.current) {
      inputRef.current.focus();
    }
    
  }, [message]);

  // 입력창 클릭 핸들러
  const handleInputClick = useCallback((e: React.MouseEvent) => {
    // 이벤트 전파 중지
    e.stopPropagation();
    
    console.log("입력창 클릭됨");
    
    // 명시적으로 포커스 설정
    if (inputRef.current) {
      inputRef.current.focus();
      console.log("입력창에 포커스 설정됨");
    }
  }, []);

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
  }, []);

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
  }, []);

  // 연결 초기화
  const initializeConnection = useCallback(() => {
    console.log("연결 초기화");
    
    // 기존 연결 정리
    if (pcRef.current) {
      pcRef.current.close();
    }
    
    // 비디오 재생 상태 초기화
    setIsVideoPlaying(false);
    
    // RTCPeerConnection 초기화
    pcRef.current = new RTCPeerConnection(pc_config);
    console.log("RTCPeerConnection 초기화 완료");
    
    // 연결 설정
    setupConnection();
  }, [setupConnection]);

  // 재연결 효과
  {/* reconnecterTrigger 값이 바뀔 때마다 자동으로 연결 초기화
    (initializeConnection()), 주로 연결 실패하거나 끊어졌을 때 재연결 위한 처리 */}
  useEffect(() => {
    if (reconnectTrigger > 0) {
      console.log("재연결 시도:", reconnectTrigger);
      initializeConnection();
    }
  }, [reconnectTrigger, initializeConnection]);

  const handleFocus = useCallback((e: React.FocusEvent<HTMLInputElement>) => {
    console.log("입력창 포커스됨");
    setInputFocused(true);
  }, []);

  const handleBlur = useCallback(() => {
    setInputFocused(false);
  }, []);

  // Enter 키 이벤트 처리
  const handleKeyPress = useCallback((e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      sendMessage();
    }
  }, [sendMessage]);
 
  // 초기 설정
  {/* 컴포넌트가 처음 렌더링될 때 Socket.IO 서버에 연결하고 각종 이벤트 리스너 설정
    연결 오류 시 에빈트 등록, 컴포넌트 사라질 때 연결을 정리*/}
  useEffect(() => {
    console.log("컴포넌트 마운트 - 연결 초기화");
    
    // Socket.IO 연결 설정
    socketRef.current = io.connect(socketServerUrl, {
      transports: ['websocket', 'polling'],
      forceNew: true,
      reconnection: true,
      reconnectionAttempts: 5,
      reconnectionDelay: 1000
    });
    console.log("Socket.IO 연결 시도");
    
    // Socket.IO 이벤트 리스너 설정
    socketRef.current.on("connect", () => {
      console.log("Socket.IO 서버에 연결됨");
      initializeConnection();
    });
    
    socketRef.current.on("connect_error", (error: any) => {
      console.error("Socket.IO 연결 오류:", error);
      setConnectionStatus("서버 연결 오류");
    });
    
    socketRef.current.on("all_users", (allUsers: Array<{ id: string }>) => {
      console.log("all_users 이벤트 수신:", allUsers);
      if (allUsers.length > 0) {
        createOffer();
      }
    });

    socketRef.current.on("getOffer", (sdp: RTCSessionDescription) => {
      console.log("getOffer 이벤트 수신");
      createAnswer(sdp);
    });

    socketRef.current.on("getAnswer", async (sdp: RTCSessionDescription) => {
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
    socketRef.current.on(
      "getCandidate",
      async (candidate: RTCIceCandidateInit) => {
        console.log("getCandidate 이벤트 수신 (바닐라 ICE에서는 일반적으로 무시됨)");
        if (!pcRef.current) return;
        
        // 호환성을 위해 후보 처리는 유지
        try {
          await pcRef.current.addIceCandidate(new RTCIceCandidate(candidate));
          console.log("ICE candidate 추가 완료");
        } catch (error) {
          console.error("ICE candidate 추가 오류:", error);
        }
      }
    );

    // 컴포넌트 마운트 후 입력창에 포커스
    setTimeout(() => {
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }, 1000);

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
  }, [setupConnection, createOffer, createAnswer, initializeConnection, playVideo]);

  // 수동 재연결 핸들러
  const handleReconnect = () => {
    setReconnectTrigger(prev => prev + 1);
  };

  return (
    <div 
      style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        height: '100vh',
        backgroundColor: '#f5f5f5',
        padding: '20px'
      }}
    >
      <h1 style={{
        textAlign: "center",
        color: "#333",
        margin: "0 0 20px 0",
        fontWeight: "bold"
      }}>RECEIVER</h1>
      
      {/* 연결 상태 표시 */}
      <div style={{
        padding: '5px 10px',
        backgroundColor: connectionStatus === "연결됨" ? '#e6f7e6' : '#ffe6e6',
        borderRadius: '4px',
        marginBottom: '10px',
        color: connectionStatus === "연결됨" ? '#2e7d32' : '#c62828',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        gap: '10px'
      }}>
        <span>{connectionStatus}</span>
        {connectionStatus !== "연결됨" && (
          <button 
            onClick={handleReconnect}
            style={{
              padding: '2px 8px',
              backgroundColor: '#f5f5f5',
              border: '1px solid #ccc',
              borderRadius: '3px',
              cursor: 'pointer',
              fontSize: '12px'
            }}
          >
            재연결
          </button>
        )}
      </div>
      
      {/* 좌우 컨텐츠를 담는 컨테이너 */}
    <div style={{
      display: 'flex',
      flexDirection: 'row', // 내부 컨텐츠는 가로 배치
      alignItems: 'flex-start',
      justifyContent: 'center',
      width: '100%',
      gap: '30px',
      flex: 1 // 남은 공간 차지
    }}>
      {/* 왼쪽 섹션: 비디오와 메시지 입력 */}
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center'
      }}>
        {/* 비디오 섹션 */}
        <div style={{ position: 'relative' }}>
          <video
            id="remotevideo"
            style={{
              width: 480,
              height: 480,
              backgroundColor: "black",
              borderRadius: "8px",
              boxShadow: "0 4px 8px rgba(0,0,0,0.1)"
            }}
            ref={remoteVideoRef}
            autoPlay
            playsInline
            muted
          />
          
          {!isVideoPlaying && (!remoteVideoRef.current?.srcObject) && (
            <button
              onClick={playVideo}
              style={{
                position: 'absolute',
                top: '50%',
                left: '50%',
                transform: 'translate(-50%, -50%)',
                padding: '12px 24px',
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                color: 'white',
                border: 'none',
                borderRadius: '8px',
                cursor: 'pointer',
                fontSize: '16px',
                fontWeight: 'bold',
                zIndex: 20
              }}
            >
              비디오 재생
            </button>
          )}
        </div>
        
        {/* 메시지 입력 영역 */}
        <div style={{
          display: 'flex',
          marginTop: '20px',
          width: '480px',
          position: 'relative',
          zIndex: 10
        }}>
          <input
            type="text"
            value={message}
            onChange={(e) => setMessage(e.target.value)}
            onKeyPress={handleKeyPress}
            onFocus={handleFocus}
            onBlur={handleBlur}
            onClick={handleInputClick}
            placeholder="메시지를 입력하세요..."
            ref={inputRef}
            style={{
              flex: 1,
              padding: '10px',
              borderRadius: '4px 0 0 4px',
              border: inputFocused ? '1px solid #4285f4' : '1px solid #ccc',
              fontSize: '16px',
              outline: 'none',
              boxShadow: inputFocused ? '0 0 0 2px rgba(66, 133, 244, 0.2)' : 'none',
              transition: 'all 0.2s ease'
            }}
            autoFocus
          />
          <button
            onClick={sendMessage}
            style={{
              padding: '10px 20px',
              backgroundColor: '#4285f4',
              color: 'white',
              border: 'none',
              borderRadius: '0 4px 4px 0',
              cursor: 'pointer',
              fontSize: '16px',
              fontWeight: 'bold',
              zIndex: 10
            }}
          >
            SEND
          </button>
        </div>
      </div>

      {/* ROS 버전 선택 */}
      <div style={{marginBottom: '20px'}}>
        <button 
          onClick={() => setRosVersion("ros1")}
          style={{
            background: rosVersion === "ros1" ? '#4285f4' : '#f5f5f5',
            color: rosVersion === "ros1"? 'white': 'black',
            padding: '8px 16px',
            border: '1px solid #ccc',
            borderRadius: '4px'
          }}
        >
          ROS1 KEY
        </button>
        <button 
          onClick={() => setRosVersion("ros2")}
          style={{
            background: rosVersion === "ros1" ? '#4285f4' : '#f5f5f5',
            color: rosVersion === "ros1"? 'white': 'black',
            padding: '8px 16px',
            border: '1px solid #ccc',
            borderRadius: '4px'
          }}
        >
          ROS2 KEY
        </button>
      </div>
      
      {/* 오른쪽 섹션: 텔레오퍼레이션 키패드 */}
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '20px',
        backgroundColor: '#ffffff',
        borderRadius: '8px',
        boxShadow: '0 4px 8px rgba(0,0,0,0.1)'
      }}>
        <TeleopKeypad // 버전 전달 수정하기
          sendRobotVelocity={sendRobotVelocity}
          movementState={movementState}
          speedLevel={speedLevel}
          setSpeedLevel={setSpeedLevel}
        />
      </div>
    </div>
  </div>
);
};

export default App;