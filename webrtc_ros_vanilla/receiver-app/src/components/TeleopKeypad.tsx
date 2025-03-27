// src/components/TeleopKeypad.tsx

import React from 'react';

interface TeleopKeypadProps {
  sendRobotVelocity: (linearX: number, angularZ: number) => void;
  movementState: string;
  speedLevel: number;
  setSpeedLevel: (callback: (prev: number) => number) => void;
}

const TeleopKeypad: React.FC<TeleopKeypadProps> = ({ 
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

  // 속도 조절 버튼 스타일
  const speedButtonStyle = {
    padding: '10px',
    backgroundColor: '#e0e0e0',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    width: '40px'
  };

  return (
    <div>
      <div style={{
        marginTop: '20px',
        width: '480px',
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
        </div>
        
        {/* teleop_key 스타일 버튼 레이아웃 */}
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: '10px' }}>
          {/* 첫 번째 행: u, i, o */}
          <button 
            onClick={() => sendRobotVelocity(1, 1)} // 왼쪽 전진
            style={getButtonStyle('왼쪽 전진')}
          >
            u
          </button>
          <button 
            onClick={() => sendRobotVelocity(1, 0)} // 직진
            style={getButtonStyle('전진')}
          >
            i
          </button>
          <button 
            onClick={() => sendRobotVelocity(1, -1)} // 오른쪽 전진
            style={getButtonStyle('오른쪽 전진')}
          >
            o
          </button>
          
          {/* 두 번째 행: j, k, l */}
          <button 
            onClick={() => sendRobotVelocity(0, 1)} // 좌회전
            style={getButtonStyle('좌회전')}
          >
            j
          </button>
          <button 
            onClick={() => sendRobotVelocity(0, 0)} // 정지
            style={{
              padding: '15px',
              backgroundColor: movementState === '정지' ? '#f44336' : '#e0e0e0',
              color: movementState === '정지' ? 'white' : 'black',
              border: 'none',
              borderRadius: '8px',
              cursor: 'pointer',
              fontWeight: 'bold'
            }}
          >
            k
          </button>
          <button 
            onClick={() => sendRobotVelocity(0, -1)} // 우회전
            style={getButtonStyle('우회전')}
          >
            l
          </button>
          
          {/* 세 번째 행: m, ',', '.' */}
          <button 
            onClick={() => sendRobotVelocity(-1, -1)} // 왼쪽 후진
            style={getButtonStyle('왼쪽 후진')}
          >
            m
          </button>
          <button 
            onClick={() => sendRobotVelocity(-1, 0)} // 후진
            style={getButtonStyle('후진')}
          >
            ,
          </button>
          <button 
            onClick={() => sendRobotVelocity(-1, 1)} // 오른쪽 후진
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
      
      {/* 도움말 */}
      <div style={{ 
        marginTop: '20px', 
        width: '480px',
        padding: '15px', 
        backgroundColor: '#f5f5f5', 
        borderRadius: '8px',
        boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
      }}>
        <h3 style={{ textAlign: 'center', margin: '0 0 10px 0', fontSize: '16px' }}>키보드 단축키</h3>
        <table style={{ width: '100%', borderCollapse: 'collapse' }}>
          <tbody>
            <tr>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>u</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>왼쪽 전진</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>i</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>전진</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>o</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>오른쪽 전진</td>
            </tr>
            <tr>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>j</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>좌회전</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>k</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>정지</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>l</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>우회전</td>
            </tr>
            <tr>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>m</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>왼쪽 후진</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>,</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>후진</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>.</td>
              <td style={{ padding: '5px', border: '1px solid #ddd' }}>오른쪽 후진</td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default TeleopKeypad;