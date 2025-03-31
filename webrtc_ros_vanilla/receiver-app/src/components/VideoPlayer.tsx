import React from "react";

interface VideoPlayerProps {
    remoteVideoRef: React.RefObject<HTMLVideoElement | null>;
    isVideoPlaying: boolean;
    playVideo: () => void;
}

const VideoPlayer: React.FC<VideoPlayerProps> = ({
    remoteVideoRef,
    isVideoPlaying,
    playVideo
}) => {
    return (
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
    );
};

export  default VideoPlayer;