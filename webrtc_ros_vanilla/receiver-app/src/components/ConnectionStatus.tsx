import React from "react";

interface ConnectionStatusProps {
    connectionStatus: string;
    handleReconnect: () => void;
}

const ConnectionStatus: React.FC<ConnectionStatusProps> = ({
    connectionStatus,
    handleReconnect
}) => {
    return (
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
    );
};

export default ConnectionStatus;