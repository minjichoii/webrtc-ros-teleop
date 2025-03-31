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
}

const RobotControlPanel: React.FC<RobotControlPanelProps> = ({
    rosVersion,
    setRosVersion,
    sendRobotVelocity,
    movementState,
    speedLevel,
    setSpeedLevel
}) => {
    return (
        <div style={{
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'flex-start',
            gap: '20px'
          }}>
            
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