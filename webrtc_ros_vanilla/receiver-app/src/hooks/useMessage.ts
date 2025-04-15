// src/hooks/useMessage.ts
import { useState, useRef, useCallback } from 'react';

interface UseMessageProps {
  onSendMessage: (message: string) => void;
  // 타입 수정: 더 유연한 참조 타입 허용
  initialRef?: React.RefObject<HTMLInputElement | null>;
}

const useMessage = ({ onSendMessage, initialRef }: UseMessageProps) => {
  // 입력창 메시지 상태 관리
  const [message, setMessage] = useState<string>("");
  // 입력창 포커스 상태
  const [inputFocused, setInputFocused] = useState<boolean>(false);
  // 입력창 참조 - 항상 useRef를 호출하고, 후에 initialRef가 있으면 그것을 사용
  const defaultRef = useRef<HTMLInputElement>(null);
  
  // 실제 사용할 ref 결정
  const effectiveRef = initialRef || defaultRef;

  // 메시지 전송 함수 - 이름 변경: sendMessage -> handleMessageSubmit
  const handleMessageSubmit = useCallback(() => {
    if (message.trim() === "") return;
    
    console.log("메시지 전송:", message);
    onSendMessage(message);
    setMessage(""); // 입력창 초기화
    
    // 메시지 전송 후 다시 입력창에 포커스
    if (effectiveRef.current) {
      effectiveRef.current.focus();
    }
  }, [message, onSendMessage, effectiveRef]);

  // 입력창 클릭 핸들러
  const handleInputClick = useCallback((e: React.MouseEvent) => {
    // 이벤트 전파 중지
    e.stopPropagation();
    
    console.log("입력창 클릭됨");
    
    // 명시적으로 포커스 설정
    if (effectiveRef.current) {
      effectiveRef.current.focus();
      console.log("입력창에 포커스 설정됨");
    }
  }, [effectiveRef]);

  // 입력창 값 변경 핸들러
  const handleInputChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    setMessage(e.target.value);
  }, []);

  // 포커스 핸들러
  const handleFocus = useCallback(() => {
    console.log("입력창 포커스됨");
    setInputFocused(true);
  }, []);

  // 블러 핸들러
  const handleBlur = useCallback(() => {
    setInputFocused(false);
  }, []);

  // Enter 키 이벤트 처리
  const handleKeyPress = useCallback((e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleMessageSubmit();
    }
  }, [handleMessageSubmit]);

  // 입력창 포커스 설정
  const focusInput = useCallback(() => {
    if (effectiveRef.current) {
      effectiveRef.current.focus();
    }
  }, [effectiveRef]);

  return {
    message,
    setMessage,
    inputFocused,
    setInputFocused, 
    inputRef: effectiveRef,
    handleInputClick,
    handleInputChange,
    handleFocus,
    handleBlur,
    handleKeyPress,
    handleMessageSubmit,
    focusInput
  };
};

export default useMessage;