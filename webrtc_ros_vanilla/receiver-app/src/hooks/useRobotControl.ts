// src/hooks/useRobotControl.ts

import { useCallback, useEffect, useRef, useState } from 'react';

interface UseRobotControlProps {
  sendRobotVelocity: (linearX: number, angularZ: number, keyInfo?: string) => void;
  isDataChannelOpen?: boolean;
  rosVersion: string;
}

interface KeyMapping {
  [key: string]: {
    linearX: number;
    angularZ: number;
    desc: string;
  };
}

/**
 * 로봇 제어를 위한 커스텀 훅
 * 키보드 및 마우스 이벤트를 처리하고 로봇 제어 명령을 관리합니다.
 */
export const useRobotControl = ({ 
  sendRobotVelocity, 
  isDataChannelOpen = true,
  rosVersion
}: UseRobotControlProps) => {
  // 현재 누르고 있는 키를 추적
  const [pressedKeys, setPressedKeys] = useState<Set<string>>(new Set());
  // 현재 누르고 있는 마우스 버튼 추적
  const [activeDirection, setActiveDirection] = useState<string | null>(null);
  // 마지막으로 전송된 명령
  const lastCommandRef = useRef<{ linearX: number, angularZ: number }>({ linearX: 0, angularZ: 0 });
  // 디버깅 정보
  const [debug, setDebug] = useState<{
    lastEvent: string, 
    isProcessing: boolean,
    inValidKeyWarning: string | null
  }>({
    lastEvent: '없음',
    isProcessing: false,
    inValidKeyWarning: null
  });

  // 잘못된 키 경고 타이머
  const warningTimerRef = useRef<NodeJS.Timeout | null>(null);

  // 키 맵핑 설정 (ROS 버전에 따라 다른 맵핑 사용 가능)
  const keyMapping = useCallback((version: 'ros1' | 'ros2'): KeyMapping => {
    if (version === 'ros1') {
      return {
        'w': { linearX: 1, angularZ: 0, desc: '전진' },
        'a': { linearX: 0, angularZ: 1, desc: '좌회전' },
        's': { linearX: 0, angularZ: 0, desc: '정지' },
        'd': { linearX: 0, angularZ: -1, desc: '우회전' }
      };
    }
    
    return {
      'u': { linearX: 1, angularZ: 1, desc: '왼쪽 전진' },
      'i': { linearX: 1, angularZ: 0, desc: '전진' },
      'o': { linearX: 1, angularZ: -1, desc: '오른쪽 전진' },
      'j': { linearX: 0, angularZ: 1, desc: '좌회전' },
      'k': { linearX: 0, angularZ: 0, desc: '정지' },
      'l': { linearX: 0, angularZ: -1, desc: '우회전' },
      'm': { linearX: -1, angularZ: -1, desc: '왼쪽 후진' },
      ',': { linearX: -1, angularZ: 0, desc: '후진' },
      '.': { linearX: -1, angularZ: 1, desc: '오른쪽 후진' }
    };
  }, []);

  // 현재 눌린 키에 따라 로봇 속도 계산
  const calculateCurrentVelocity = useCallback((keys: Set<string>, version: 'ros1' | 'ros2') => {
    const mapping = keyMapping(version);
    let linearX = 0;
    let angularZ = 0;
    let activeKeys = '';
    
    // 정지 키('s' 또는 'k')가 눌려있으면 즉시 정지
    if ((version === 'ros1' && keys.has('s')) || (version === 'ros2' && keys.has('k'))) {
      return { linearX: 0, angularZ: 0, keyInfo: version === 'ros1' ? 's' : 'k' };
    }

    if (version === 'ros1') {
      // ROS1 w,a,d 키 조합
      if (keys.has('w')) {
        linearX = 1;
        activeKeys += 'w';
      }
      if (keys.has('a')) {
        angularZ = 1;
        activeKeys += 'a';
      }
      if (keys.has('d')) {
        angularZ = -1;
        activeKeys += 'd';
      }
    } else {
      // ROS2 
      if (keys.has('i')) {
        linearX = 1;
        activeKeys += 'i';
      }
      if (keys.has(',')) {
        linearX = -1;
        activeKeys += ',';
      }
      if (keys.has('j')) {
        angularZ = 1;
        activeKeys += 'j';
      }
      if (keys.has('l')) {
        angularZ = -1;
        activeKeys += 'l';
      }

      // 대각선 이동키
      if (linearX === 0 && angularZ === 0) {
        if (keys.has('u')) {
          linearX = 1;
          angularZ = 1;
          activeKeys = 'u';
        } else if (keys.has('o')) {
          linearX = 1;
          angularZ = -1;
          activeKeys = 'o';
        } else if (keys.has('m')) {
          linearX = -1;
          angularZ = -1;
          activeKeys = 'm';
        } else if (keys.has('.')) {
          linearX = -1;
          angularZ = 1;
          activeKeys = '.';
        }
      }
    }
    
    return { linearX, angularZ, keyInfo: activeKeys };
  }, [keyMapping]);

  // 경고 메시지 표시 함수
  const showInvalidKeyWarning = useCallback((key: string) => {
    if (warningTimerRef.current) {
      clearTimeout(warningTimerRef.current);
    }

    const validKeys = Object.keys(keyMapping(rosVersion as 'ros1' | 'ros2')).join(', ');

    setDebug(prev => ({
      ...prev,
      inValidKeyWarning: `'${key}' 키는 현재 모드(${rosVersion})에서 사용할 수 없습니다. 유효한 키: ${validKeys}`
    }));

    warningTimerRef.current = setTimeout(() => {
      setDebug(prev => ({
        ...prev,
        inValidKeyWarning: null
      }));
      warningTimerRef.current = null;
    }, 3000);
  }, [rosVersion, keyMapping]);

  // 키보드 이벤트 핸들러
  useEffect(() => {
    if (!isDataChannelOpen) return;
    
    const handleKeyDown = (e: KeyboardEvent) => {
      // 입력 필드에 포커스가 있는 경우 키보드 컨트롤 비활성화
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) {
        return;
      }
      
      const key = e.key.toLowerCase();
      
      // 이미 누른 키는 무시 (키 반복 방지)
      if (pressedKeys.has(key)) return;

      const mapping = keyMapping(rosVersion as 'ros1' | 'ros2');
      if (!mapping[key]) {
        showInvalidKeyWarning(key);
        return;
      }
      
      // 새 키 추가
      const newKeys = new Set(pressedKeys);
      newKeys.add(key);
      setPressedKeys(newKeys);
      
      const { linearX, angularZ, keyInfo } = calculateCurrentVelocity(newKeys, rosVersion as 'ros1' | 'ros2');
      
      // 마지막 명령과 다른 경우에만 전송
      if (lastCommandRef.current.linearX !== linearX || 
          lastCommandRef.current.angularZ !== angularZ) {
        sendRobotVelocity(linearX, angularZ, `키보드: ${keyInfo}`);
        lastCommandRef.current = { linearX, angularZ };
        
        setDebug(prev => ({
          ...prev,
          lastEvent: `키보드 다운: ${keyInfo} => [${linearX}, ${angularZ}]`
        }));
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      // 입력 필드에 포커스가 있는 경우 키보드 컨트롤 비활성화
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) {
        return;
      }
      
      const key = e.key.toLowerCase();
      
      // 키가 눌려있지 않으면 무시
      if (!pressedKeys.has(key)) return;
      
      // 키 제거
      const newKeys = new Set(pressedKeys);
      newKeys.delete(key);
      setPressedKeys(newKeys);

      if (newKeys.size === 0) {
        // 모든 키를 놓았으면 정지
        sendRobotVelocity(0, 0, `키보드 업: ${key} (정지)`);
        lastCommandRef.current = { linearX: 0, angularZ: 0 };

        setDebug(prev => ({
          ...prev,
          lastEvent: `키보드 업: 모든 키 => 정지`
        }));
      } else {
        const { linearX, angularZ, keyInfo } = calculateCurrentVelocity(newKeys, rosVersion as 'ros1' | 'ros2');
        sendRobotVelocity(linearX, angularZ, `키보드 업: ${key}, 남은 키: ${keyInfo}`);
        lastCommandRef.current = {linearX, angularZ};

        setDebug(prev => ({
          ...prev,
          lastEvent: `키보드 업: ${key}, 남은 키: ${keyInfo} => [${linearX}, ${angularZ}]`
        }));
      }
    };

    // 윈도우 포커스 손실 시 모든 키 초기화
    const handleBlur = () => {
      if (pressedKeys.size > 0) {
        setPressedKeys(new Set());
        sendRobotVelocity(0, 0);
        lastCommandRef.current = { linearX: 0, angularZ: 0 };
        
        setDebug(prev => ({
          ...prev,
          lastEvent: '윈도우 포커스 손실 => 정지'
        }));
      }
    };

    // 이벤트 리스너 등록
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.addEventListener('blur', handleBlur);
    
    // 컴포넌트 언마운트 시 이벤트 리스너 제거
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      window.removeEventListener('blur', handleBlur);
    };
  }, [pressedKeys, calculateCurrentVelocity, sendRobotVelocity, isDataChannelOpen, rosVersion]);

  // 마우스 이벤트 핸들러
  const handleDirectionMouseDown = useCallback((linearX: number, angularZ: number, direction: string) => {
    if (!isDataChannelOpen) return;
    
    setActiveDirection(direction);
    sendRobotVelocity(linearX, angularZ, `마우스: ${direction}`);
    lastCommandRef.current = { linearX, angularZ };
    
    setDebug(prev => ({
      ...prev,
      lastEvent: `마우스 다운: ${direction} => [${linearX}, ${angularZ}]`
    }));
    
    // 윈도우 레벨 이벤트 리스너 추가 (버튼 외부로 마우스가 이동했을 때도 감지)
    const handleGlobalMouseUp = () => {
      if (activeDirection) {
        setActiveDirection(null);
        sendRobotVelocity(0, 0, `마우스 업: ${direction}`);
        lastCommandRef.current = { linearX: 0, angularZ: 0 };
        
        setDebug(prev => ({
          ...prev,
          lastEvent: `전역 마우스 업: ${direction} => 정지`
        }));
        
        // 이벤트 리스너 제거
        window.removeEventListener('mouseup', handleGlobalMouseUp);
      }
    };
    
    window.addEventListener('mouseup', handleGlobalMouseUp);
  }, [sendRobotVelocity, activeDirection, isDataChannelOpen]);

  // 직접 정지 명령 전송
  const sendStopCommand = useCallback(() => {
    if (!isDataChannelOpen) return;
    
    setActiveDirection(null);
    setPressedKeys(new Set());
    sendRobotVelocity(0, 0, `수동 정지 명령`);
    lastCommandRef.current = { linearX: 0, angularZ: 0 };
    
    setDebug(prev => ({
      ...prev,
      lastEvent: '수동 정지 명령'
    }));
  }, [sendRobotVelocity, isDataChannelOpen]);

  return {
    handleDirectionMouseDown,
    sendStopCommand,
    activeDirection,
    debug
  };
};