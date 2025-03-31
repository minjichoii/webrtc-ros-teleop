import React from "react";

interface MessageInputProps {
    message: string;
    onChange: (e: React.ChangeEvent<HTMLInputElement>) => void;
    onKeyPress: (e: React.KeyboardEvent<HTMLInputElement>) => void;
    onFocus: React.FocusEventHandler<HTMLInputElement>;
    onBlur: React.FocusEventHandler<HTMLInputElement>;
    onClick: (e: React.MouseEvent) => void;
    onSend: () => void;
    inputRef: { current: HTMLInputElement | null };
    isFocused: boolean;
}

const MessageInput: React.FC<MessageInputProps> = ({
    message,
    onChange,
    onKeyPress,
    onFocus,
    onBlur,
    onClick,
    onSend,
    inputRef,
    isFocused
}) => {
    return (
        <div style={{
            display: 'flex',
            marginTop: '20px',
            width: '480px',
            position: 'relative',
            zIndex: 10
          }}>
            <input
              type="text"
              value={message}
              onChange={onChange}
              onKeyPress={onKeyPress}
              onFocus={onFocus}
              onBlur={onBlur}
              onClick={onClick}
              placeholder="메시지를 입력하세요..."
              ref={inputRef}
              style={{
                flex: 1,
                padding: '10px',
                borderRadius: '4px 0 0 4px',
                border: isFocused ? '1px solid #4285f4' : '1px solid #ccc',
                fontSize: '16px',
                outline: 'none',
                boxShadow: isFocused ? '0 0 0 2px rgba(66, 133, 244, 0.2)' : 'none',
                transition: 'all 0.2s ease'
              }}
              autoFocus
            />
            <button
              onClick={onSend}
              style={{
                padding: '10px 20px',
                backgroundColor: '#4285f4',
                color: 'white',
                border: 'none',
                borderRadius: '0 4px 4px 0',
                cursor: 'pointer',
                fontSize: '16px',
                fontWeight: 'bold',
                zIndex: 10
              }}
            >
              SEND
            </button>
          </div>
    );
}

export default MessageInput;