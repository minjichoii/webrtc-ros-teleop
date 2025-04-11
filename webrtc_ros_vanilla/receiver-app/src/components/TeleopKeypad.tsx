// src/components/TeleopKeypad.tsx

import React from 'react';
import { useEffect, useRef, useState } from 'react';
import { useRobotControl } from '../hooks/useRobotControl';

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
  const [lastEvent, setLastEvent] = useState<string>('없음');

  const { handleDirectionMouseDown, sendStopCommand, activeDirection, debug } = useRobotControl({
    sendRobotVelocity,
    isDataChannelOpen: true,
    rosVersion
  });

  useEffect(() => {
    setLastEvent(debug.lastEvent);
  }, [debug.lastEvent]);

  // 버튼 스타일 함수 - 현재 상태에 따라 색상 변경
  const getButtonStyle = (direction: string) => ({
    padding: '15px',
    backgroundColor: activeDirection === direction
      ? '#4285f4'
      : (movementState === direction ? '#2196f3' : '#e0e0e0'),
    color: (activeDirection === direction || movementState === direction) ? 'white' : 'black',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    fontWeight: 'bold' as const,
    userSelect: 'none' as const,
    transition: 'all 0.2s ease'
  });

  // 정지 버튼 스타일
  const stopButtonStyle = {
    padding: '15px',
    backgroundColor: movementState === '정지' ? '#f44336' : '#e0e0e0',
    color: movementState === '정지' ? 'white' : 'black',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
    fontWeight: 'bold' as const,
    userSelect: 'none' as const,
    transition:  'all 0.2 ease'
  };

  // 속도 조절 버튼 스타일
  const speedButtonStyle = {
    padding: '10px',
    backgroundColor: '#e0e0e0',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    width: '40px',
    userSelect: 'none' as const
  };

  // 경고 메시지
  const KeyWarning = () => {
    if (!debug.inValidKeyWarning) return null;

    return (
      <div style={{
        position: 'fixed',
        bottom: '20px',
        left: '50%',
        transform: 'translateX(-50%)',
        backgroundColor: 'rgba(255, 0, 0, 0.8)',
        color: 'white',
        padding: '10px 20px',
        borderRadius: '4px',
        boxShadow: '0 2px 8px rgba(0, 0, 0, 0.2)',
        zIndex: 1000,
        maxWidth: '90%',
        textAlign: 'center',
        fontSize: '14px',
        fontWeight: 'bold'
      }}>
        {debug.inValidKeyWarning}
      </div>
    );
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
              onMouseDown={() => handleDirectionMouseDown(1,0, '전진')}
              style={getButtonStyle('전진')}
            >
              w
            </button>
            <div></div> {/* 빈 셀 */}
            
            {/* 두 번째 행 */}
            <button 
              onMouseDown={() => handleDirectionMouseDown(0, 1, '좌회전')}
              style={getButtonStyle('좌회전')}
            >
              a
            </button>
            <button 
              onClick={sendStopCommand}
              style={stopButtonStyle}
            >
              s
            </button>
            <button 
              onMouseDown={() => handleDirectionMouseDown(0, -1, '우회전')}
              style={getButtonStyle('우회전')}
            >
              d
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
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center', width: '15%' }}>w</td>
                <td style={{ padding: '8px', border: '1px solid #ddd', width: '35%' }}>w</td>
              </tr>
              <tr>
              <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center' }}>a</td>
                <td style={{ padding: '8px', border: '1px solid #ddd' }}>좌회전</td>
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center', width: '15%' }}>s</td>
                <td style={{ padding: '8px', border: '1px solid #ddd', width: '35%' }}>정지</td>
              </tr>
              <tr>
                <td style={{ padding: '8px', border: '1px solid #ddd', textAlign: 'center' }}>d</td>
                <td style={{ padding: '8px', border: '1px solid #ddd' }}>우회전</td>
              </tr>
            </tbody>
          </table>
          <div style={{marginTop: '10px', fontSize: '14px', color: '#555'}}>
            * 키보드 키를 누르고 있으면 로봇이 움직이고, 키를 놓으면 자동 정지
          </div>
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
              onMouseDown={() => handleDirectionMouseDown(1, 1, '왼쪽 전진')}
              style={getButtonStyle('왼쪽 전진')}
            >
              u
            </button>
            <button 
              onMouseDown={() => handleDirectionMouseDown(1, 0, '전진')}
              style={getButtonStyle('전진')}
            >
              i
            </button>
            <button 
              onMouseDown={() => handleDirectionMouseDown(1, -1, '오른쪽 전진')}
              style={getButtonStyle('오른쪽 전진')}
            >
              o
            </button>
            
            {/* 두 번째 행 */}
            <button 
              onMouseDown={() => handleDirectionMouseDown(0, 1, '좌회전')}
              style={getButtonStyle('좌회전')}
            >
              j
            </button>
            <button 
              onClick={sendStopCommand}
              style={stopButtonStyle}
            >
              k
            </button>
            <button 
              onMouseDown={() => handleDirectionMouseDown(0, -1, '우회전')}
              style={getButtonStyle('우회전')}
            >
              l
            </button>
            
            {/* 세 번째 행 */}
            <button 
              onMouseDown={() => handleDirectionMouseDown(-1, -1, '왼쪽 후진')}
              style={getButtonStyle('왼쪽 후진')}
            >
              m
            </button>
            <button 
              onMouseDown={() => handleDirectionMouseDown(-1, 0, '후진')}
              style={getButtonStyle('후진')}
            >
              ,
            </button>
            <button 
              onMouseDown={() => handleDirectionMouseDown(-1, 1, '오른쪽 후진')}
              style={getButtonStyle('오른쪽 후진')}
            >
              .
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
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>u</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>왼쪽 전진</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>i</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>전진</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>o</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>오른쪽 전진</td>
              </tr>
              <tr>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>j</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>좌회전</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>k</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>정지</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>l</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>우회전</td>
              </tr>
              <tr>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>m</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>왼쪽 후진</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>,</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>후진</td>
                <td style={{ padding: '5px', border: '1px solid #ddd', textAlign: 'center' }}>.</td>
                <td style={{ padding: '5px', border: '1px solid #ddd' }}>오른쪽 후진</td>
              </tr>
            </tbody>
          </table>
          <div style={{ marginTop: '10px', fontSize: '14px', color: '#555' }}>
            ※ 키보드 키를 누르고 있으면 로봇이 움직이고, 키를 놓으면 자동으로 정지합니다.
          </div>
        </div>
      </div>
    );
  }
};

export default TeleopKeypad;