import React from "react";

interface RosVersionSelectProps {
    rosVersion: string;
    setRosVersion: (version: string) => void;
}

const RosVersionSelect: React.FC<RosVersionSelectProps> = ({ rosVersion, setRosVersion }) => {
    // 버튼 기본 스타일
    const baseButtonStyle = {
        padding: '12px 16px',
        border: '1px solid #ccc',
        borderRadius: '4px',
        flex: '1',
        cursor: 'pointer',
        fontWeight: 'bold' as const,
        fontSize: '16px',
        transition: 'all 0.2s ease'
    };

    // 활성화된 버튼 스타일
    const activeButtonStyle = {
        ...baseButtonStyle,
        backgroundColor: '#4285f4',
        color: 'white',
        boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
    };

    // 비활성화된 버튼 스타일
    const inactiveButtonStyle = {
        ...baseButtonStyle,
        backgroundColor: '#f5f5f5',
        color: '#333'
    };

    return (
        <div style={{
            display: 'flex',
            flexDirection: 'row',
            justifyContent: 'space-between',
            gap: '10px',
            width: '100%'
          }}>
            <button 
              onClick={() => setRosVersion("ros1")}
              style={{
                background: rosVersion === "ros1" ? '#4285f4' : '#f5f5f5',
                color: rosVersion === "ros1" ? 'white' : 'black',
                padding: '8px 16px',
                border: '1px solid #ccc',
                borderRadius: '4px',
                flex: '1'
              }}
            >
              ROS1 KEY
            </button>
            <button 
              onClick={() => setRosVersion("ros2")}
              style={{
                background: rosVersion === "ros2" ? '#4285f4' : '#f5f5f5',
                color: rosVersion === "ros2" ? 'white' : 'black',
                padding: '8px 16px',
                border: '1px solid #ccc',
                borderRadius: '4px',
                flex: '1'
              }}
            >
              ROS2 KEY
            </button>
          </div>
    );
};

export default RosVersionSelect;