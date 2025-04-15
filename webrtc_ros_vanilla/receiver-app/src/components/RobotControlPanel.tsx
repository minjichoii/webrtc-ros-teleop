import React from "react";
import TeleopKeypad from "./TeleopKeypad";
import RosVersionSelect from "./RosVersionSelect";

interface RobotControlPanelProps {
    rosVersion: string;
    setRosVersion: React.Dispatch<React.SetStateAction<string>>;
    sendRobotVelocity: (linearX: number, angularZ: number) => void;
    movementState: string;
    speedLevel: number;
    setSpeedLevel: React.Dispatch<React.SetStateAction<number>>;
    dataChannelReady?: boolean;
}

const RobotControlPanel: React.FC<RobotControlPanelProps> = ({
    rosVersion,
    setRosVersion,
    sendRobotVelocity,
    movementState,
    speedLevel,
    setSpeedLevel,
    dataChannelReady=false
}) => {
    return (
        <div style={{
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'flex-start',
            gap: '20px'
          }}>
            
            {/* 데이터 채널 상태 표시 */}
            <div style={{
                backgroundColor: dataChannelReady ? '#e6f7e6' : '#fff3e6',
                borderRadius: '5px',
                padding: '8px 12px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                width: '100%'
            }}>
                <div style={{
                    width: '10px',
                    height: '10px',
                    borderRadius: '50%',
                    backgroundColor: dataChannelReady ? '#4CAF50' : '#FF9800',
                    marginRight: '8px'
                }} />
                <span style={{ fontSize: '14px' }}>
                    {dataChannelReady ? '제어 채널 준비됨' : '제어 채널 연결 중...'}
                </span>
            </div>

            <RosVersionSelect
              rosVersion={rosVersion}
              setRosVersion={setRosVersion}
            />
            
            {/* 오른쪽 섹션: 텔레오퍼레이션 키패드 */}
            <div style={{
              backgroundColor: '#ffffff',
              borderRadius: '8px',
              boxShadow: '0 4px 8px rgba(0,0,0,0.1)',
              padding: '20px'
            }}>
              <TeleopKeypad // 버전 전달 수정하기
                rosVersion={rosVersion}
                sendRobotVelocity={sendRobotVelocity}
                movementState={movementState}
                speedLevel={speedLevel}
                setSpeedLevel={setSpeedLevel}
              />
            </div>
          </div>
    );
};

export default RobotControlPanel;