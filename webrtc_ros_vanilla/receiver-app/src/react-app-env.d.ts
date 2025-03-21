/// <reference types="react-scripts" />

declare namespace NodeJS {
    interface ProcessEnv {
      REACT_APP_SOCKET_SERVER_URL: string;
      REACT_APP_ROOM_ID: string;
    }
  }
  