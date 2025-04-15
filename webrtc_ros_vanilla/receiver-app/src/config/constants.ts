// WebRTC 설정
export const PC_CONFIG = {
    iceServers: [
      {
        urls: "stun:stun.l.google.com:19302",
      },
    ],
};

export const SOCKET_SERVER_URL = process.env.REACT_APP_SOCKET_SERVER_URL;
export const ROOM_ID = process.env.REACT_APP_ROOM_ID;

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
 
export const BASE_SPEED = 0.2; 
export const MAX_SPEED = 1.0;

export const ICE_GATHERING_TIMEOUT = 10000;
export const FOCUS_DELAY = 500;
export const RECONNECT_DELAY = 5000;