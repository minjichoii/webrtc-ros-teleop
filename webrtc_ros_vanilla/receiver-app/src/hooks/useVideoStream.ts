// src/hooks/useVideoStream.ts
import { useRef, useCallback, useState } from 'react';

// VideoStream 훅은 비디오 스트림 및 재생 관련 기능을 제공합니다.
const useVideoStream = () => {
  // <video> DOM 요소에 접근해서 원격 스트림 표시하기 위해 사용
  const remoteVideoRef = useRef<HTMLVideoElement>(null);
  // 비디오 재생 상태
  const [isVideoPlaying, setIsVideoPlaying] = useState<boolean>(false);

  // 비디오 재생 시도
  const playVideo = useCallback(() => {
    if (remoteVideoRef.current) {
      console.log("비디오 재생 시도");
      console.log("현재 srcObject 상태:", remoteVideoRef.current.srcObject ? "있음" : "없음");
      console.log("현재 재생 상태:", isVideoPlaying ? "재생 중" : "재생 안 됨");
      
      // 스트림이 없으면 스트림 확인 메시지 표시
      if (!remoteVideoRef.current.srcObject) {
        console.warn("비디오 스트림이 없습니다. 연결 상태를 확인하세요.");
        alert("비디오 스트림이 없습니다. 연결 상태를 확인하세요.");
        return;
      }
  
      // 항상 mute 상태로 먼저 시도 (자동 재생 정책 우회)
      remoteVideoRef.current.muted = true;
      
      // 재생 시도
      remoteVideoRef.current.play()
        .then(() => {
          console.log("비디오 재생 성공");
          setIsVideoPlaying(true);
          
          // 1초 후 음소거 해제 시도 (필요한 경우)
          setTimeout(() => {
            if (remoteVideoRef.current) {
              // 사용자 상호작용이 있었으므로 음소거 해제 가능
              // remoteVideoRef.current.muted = false;  // 필요에 따라 주석 해제
            }
          }, 1000);
        })
        .catch(e => {
          console.error("비디오 재생 오류:", e);
          setIsVideoPlaying(false);
          
          // 자동 재생 정책 관련 오류 메시지 표시
          alert("비디오 재생에 실패했습니다. 브라우저의 자동 재생 정책으로 인해 제한될 수 있습니다. 다시 시도해주세요.");
        });
    }
  }, [isVideoPlaying]);

  // 원격 트랙 수신 처리 - App.tsx에서 직접 처리하므로 여기서는 제거함
  const handleTrackEvent = useCallback((event: RTCTrackEvent) => {
    // 이 함수는 원본 코드에서 App.tsx가 직접 처리하는 ontrack 이벤트 핸들러입니다.
    // 실제로는 이 함수를 사용하지 않고, App.tsx에서 직접 처리합니다.
    console.log("원격 트랙 이벤트 수신");
  }, []);

  // 비디오 상태 초기화
  const resetVideoState = useCallback(() => {
    setIsVideoPlaying(false);
  }, []);

  return {
    remoteVideoRef,
    isVideoPlaying,
    setIsVideoPlaying,
    handleTrackEvent,
    playVideo,
    resetVideoState
  };
};

export default useVideoStream;