// src/components/TeleopKeypad.tsx

import React from 'react';

interface TeleopKeypadProps {
  rosVersion: string;
  sendRobotVelocity: (linearX: number, angularZ: number) => void;
  movementState: string;
  speedLevel: number;
  setSpeedLevel: (callback: (prev: number) => number) => void;
}

const TeleopKeypad: React.FC<TeleopKeypadProps> = ({ 
  rosVersion,
  sendRobotVelocity, 
  movementState, 
  speedLevel, 
  setSpeedLevel 
}) => {
  // 버튼 스타일 함수 - 현재 상태에 따라 색상 변경
  const getButtonStyle = (state: string) => ({
    padding: '15px',
    backgroundColor: movementState === state ? '#4285f4' : '#e0e0e0',
    color: movementState === state ? 'white' : 'black',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    fontWeight: 'bold' as const
  });

  // 정지 버튼 스타일
  const stopButtonStyle = {
    padding: '15px',
    backgroundColor: movementState === '정지' ? '#f44336' : '#e0e0e0',
    color: movementState === '정지' ? 'white' : 'black',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    fontWeight: 'bold' as const
  };

  // 속도 조절 버튼 스타일
  const speedButtonStyle = {
    padding: '10px',
    backgroundColor: '#e0e0e0',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    width: '40px'
  };

  // ROS1용 키 매핑 (WASD 스타일 - 4개 키)
  const ros1Mapping = {
    top: { key: 'w', action: () => sendRobotVelocity(1, 0), desc: '전진' },
    left: { key: 'a', action: () => sendRobotVelocity(0, 1), desc: '좌회전' },
    stop: { key: 's', action: () => sendRobotVelocity(0, 0), desc: '정지' },
    right: { key: 'd', action: () => sendRobotVelocity(0, -1), desc: '우회전' }
  };

  // ROS2용 키 매핑 (UIOJKL,. - 9개 키)
  const ros2Mapping = {
    topLeft: { key: 'u', action: () => sendRobotVelocity(1, 1), desc: '왼쪽 전진' },
    top: { key: 'i', action: () => sendRobotVelocity(1, 0), desc: '전진' },
    topRight: { key: 'o', action: () => sendRobotVelocity(1, -1), desc: '오른쪽 전진' },
    left: { key: 'j', action: () => sendRobotVelocity(0, 1), desc: '좌회전' },
    stop: { key: 'k', action: () => sendRobotVelocity(0, 0), desc: '정지' },
    right: { key: 'l', action: () => sendRobotVelocity(0, -1), desc: '우회전' },
    bottomLeft: { key: 'm', action: () => sendRobotVelocity(-1, -1), desc: '왼쪽 후진' },
    bottom: { key: ',', action: () => sendRobotVelocity(-1, 0), desc: '후진' },
    bottomRight: { key: '.', action: () => sendRobotVelocity(-1, 1), desc: '오른쪽 후진' }
  };

  // 현재 선택된 ROS 버전에 따라 다른 키패드 렌더링
  if (rosVersion === 'ros1') {
    // ROS1 - WASD 스타일 키패드 (4개 키)
    return (
      <div>
        <div style={{
          width: '100%',
          backgroundColor: '#f0f0f0',
          borderRadius: '8px',
          padding: '15px',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
        }}>
          <div style={{ textAlign: 'center', marginBottom: '10px' }}>
            <div style={{ fontWeight: 'bold', fontSize: '18px' }}>로봇 제어</div>
            <div style={{ color: '#555', fontSize: '14px' }}>
              현재 상태: <span style={{ fontWeight: 'bold' }}>{movementState || '정지'}</span>
            </div>
            <div style={{ color: '#555', fontSize: '14px', marginTop: '5px' }}>
              현재 키 레이아웃: <span style={{ fontWeight: 'bold' }}>WASD (ROS1)</span>
            </div>
          </div>
          
          {/* WASD 키패드 레이아웃 */}
          <div style={{ 
            display: 'grid', 
            gridTemplateColumns: '1fr 1fr 1fr', 
            gridTemplateRows: 'auto auto',
            gap: '10px',
            width: '100%',
            maxWidth: '300px',
            margin: '0 auto'
          }}>
            {/* 첫 번째 행 */}
            <div></div> {/* 빈 셀 */}
            <button 
              onClick={ros1Mapping.top.action}
              style={getButtonStyle(ros1Mapping.top.desc)}
            >
              {ros1Mapping.top.key}
            </button>
            <div></div> {/* 빈 셀 */}
            
            {/* 두 번째 행 */}
            <button 
              onClick={ros1Mapping.left.action}
              style={getButtonStyle(ros1Mapping.left.desc)}
            >
              {ros1Mapping.left.key}
            </button>
            <button 
              onClick={ros1Mapping.stop.action}
              style={stopButtonStyle}
            >
              {ros1Mapping.stop.key}
            </button>
            <button 
              onClick={ros1Mapping.right.action}
              style={getButtonStyle(ros1Mapping.right.desc)}
            >
              {ros1Mapping.right.key}
            </button>
          </div>
          
          {/* 속도 제어 */}
          <div style={{ 
            marginTop: '15px', 
            display: 'flex', 
            justifyContent: 'space-between',
            alignItems: 'center' 
          }}>
            <button 
              onClick={() => setSpeedLevel(prev => Math.max(prev - 1, 1))}
              style={speedButtonStyle}
            >
              -
            </button>
            <div style={{ 
              flex: 1, 
              textAlign: 'center',
              fontSize: '14px'
            }}>
              속도: {speedLevel}/10
            </div>
            <button 
              onClick={() => setSpeedLevel(prev => Math.min(prev + 1, 10))}
              style={speedButtonStyle}
            >
              +
            </button>
          </div>
        </div>
        
        {/* 키보드 단축키 설명 */}
        <div style={{ 
          marginTop: '20px', 
          width: '100%',
          padding: '15px', 
          backgroundColor: '#f5f5f5', 
          borderRadius: '8px',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
        }}>
          <h3 style={{ textAlign: 'center', margin: '0 0 10px 0', fontSize: '16px' }}>키보드 단축키</h3>
          <table style={{ width: '100%', borderCollapse: 'collapse' }}>
            <tbody>
              <tr>
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center', width: '15%' }}>{ros1Mapping.top.key}</td>
                <td style={{ padding: '8px', border: '1px solid #ddd', width: '35%' }}>{ros1Mapping.top.desc}</td>
              </tr>
              <tr>
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center' }}>{ros1Mapping.left.key}</td>
                <td style={{ padding: '8px', border: '1px solid #ddd' }}>{ros1Mapping.left.desc}</td>
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center', width: '15%' }}>{ros1Mapping.stop.key}</td>
                <td style={{ padding: '8px', border: '1px solid #ddd', width: '35%' }}>{ros1Mapping.stop.desc}</td>
              </tr>
              <tr>
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center' }}>{ros1Mapping.right.key}</td>
                <td style={{ padding: '8px', border: '1px solid #ddd' }}>{ros1Mapping.right.desc}</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    );
  } else {
    // ROS2 - UIOJKL,. 키패드 (9개 키)
    return (
      <div>
        <div style={{
          width: '100%',
          backgroundColor: '#f0f0f0',
          borderRadius: '8px',
          padding: '15px',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
        }}>
          <div style={{ textAlign: 'center', marginBottom: '10px' }}>
            <div style={{ fontWeight: 'bold', fontSize: '18px' }}>로봇 제어</div>
            <div style={{ color: '#555', fontSize: '14px' }}>
              현재 상태: <span style={{ fontWeight: 'bold' }}>{movementState || '정지'}</span>
            </div>
            <div style={{ color: '#555', fontSize: '14px', marginTop: '5px' }}>
              현재 키 레이아웃: <span style={{ fontWeight: 'bold' }}>UIOJKL,. (ROS2)</span>
            </div>
          </div>
          
          {/* 9-키 레이아웃 */}
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '10px' }}>
            {/* 첫 번째 행 */}
            <button 
              onClick={ros2Mapping.topLeft.action}
              style={getButtonStyle(ros2Mapping.topLeft.desc)}
            >
              {ros2Mapping.topLeft.key}
            </button>
            <button 
              onClick={ros2Mapping.top.action}
              style={getButtonStyle(ros2Mapping.top.desc)}
            >
              {ros2Mapping.top.key}
            </button>
            <button 
              onClick={ros2Mapping.topRight.action}
              style={getButtonStyle(ros2Mapping.topRight.desc)}
            >
              {ros2Mapping.topRight.key}
            </button>
            
            {/* 두 번째 행 */}
            <button 
              onClick={ros2Mapping.left.action}
              style={getButtonStyle(ros2Mapping.left.desc)}
            >
              {ros2Mapping.left.key}
            </button>
            <button 
              onClick={ros2Mapping.stop.action}
              style={stopButtonStyle}
            >
              {ros2Mapping.stop.key}
            </button>
            <button 
              onClick={ros2Mapping.right.action}
              style={getButtonStyle(ros2Mapping.right.desc)}
            >
              {ros2Mapping.right.key}
            </button>
            
            {/* 세 번째 행 */}
            <button 
              onClick={ros2Mapping.bottomLeft.action}
              style={getButtonStyle(ros2Mapping.bottomLeft.desc)}
            >
              {ros2Mapping.bottomLeft.key}
            </button>
            <button 
              onClick={ros2Mapping.bottom.action}
              style={getButtonStyle(ros2Mapping.bottom.desc)}
            >
              {ros2Mapping.bottom.key}
            </button>
            <button 
              onClick={ros2Mapping.bottomRight.action}
              style={getButtonStyle(ros2Mapping.bottomRight.desc)}
            >
              {ros2Mapping.bottomRight.key}
            </button>
          </div>
          
          {/* 속도 제어 */}
          <div style={{ 
            marginTop: '15px', 
            display: 'flex', 
            justifyContent: 'space-between',
            alignItems: 'center' 
          }}>
            <button 
              onClick={() => setSpeedLevel(prev => Math.max(prev - 1, 1))}
              style={speedButtonStyle}
            >
              -
            </button>
            <div style={{ 
              flex: 1, 
              textAlign: 'center',
              fontSize: '14px'
            }}>
              속도: {speedLevel}/10
            </div>
            <button 
              onClick={() => setSpeedLevel(prev => Math.min(prev + 1, 10))}
              style={speedButtonStyle}
            >
              +
            </button>
          </div>
        </div>
        
        {/* 키보드 단축키 도움말 */}
        <div style={{ 
          marginTop: '20px', 
          width: '100%',
          padding: '15px', 
          backgroundColor: '#f5f5f5', 
          borderRadius: '8px',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
        }}>
          <h3 style={{ textAlign: 'center', margin: '0 0 10px 0', fontSize: '16px' }}>키보드 단축키</h3>
          <table style={{ width: '100%', borderCollapse: 'collapse' }}>
            <tbody>
              <tr>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.topLeft.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.topLeft.desc}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.top.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.top.desc}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.topRight.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.topRight.desc}</td>
              </tr>
              <tr>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.left.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.left.desc}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.stop.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.stop.desc}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.right.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.right.desc}</td>
              </tr>
              <tr>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.bottomLeft.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.bottomLeft.desc}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.bottom.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.bottom.desc}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>{ros2Mapping.bottomRight.key}</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>{ros2Mapping.bottomRight.desc}</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    );
  }
};

export default TeleopKeypad;