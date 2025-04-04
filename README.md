# WebRTC ROS Teleoperation - Vanilla ICE Mode

## 📁 프로젝트 구조
```
webrtc_ros_vanilla/
├── receiver-app/       # React 앱 (영상 수신 및 표시, 기존 코드 ice candidate 교환 방식만 수정)
├── server.signaling/   # Signaling 서버 (Node.js 기반, 기존 코드)
├── sender_vanilla.py   # Python 송신기 (ROS 카메라 스트림 전송 및 송출)
```

---

## ✍️ 프로젝트 설명
```
ROS에서 송출되는 영상을 WebRTC를 통해 React 앱에서 실시간으로 모니터링용 
```
기존 트리클 ICE 방식에서 **Vanilla ICE 방식**으로 변경하여 연결 및 통신 방식을 단순화하였습니다. 
(트리클 방식에서는 ICE 후보 교환이 안되는 문제가 있었음)

---

## 📅 업데이트 기록
### ✅ 2025-03-21

#### ❇️ 수행 내역
- 기존 트리클 ICE 방식에서 **Vanilla ICE 방식으로 변경**
- React 앱에서 영상 스트림 수신 성공 (29~30 fps) 😄 **실시간성**

#### 🔨 추가해야 할 점
- 재연결 시 발생하는 오류 수정 필요
- 연결 안정성 개선 및 성능 최적화
- 윗부분 다 수정한 뒤, ROS 이동 명령어 주고받는 부분 구현 시작

### ✅ 2025-03-27

#### ❇️ 수행 내역
- teleop keypad UI 제작
- teleop keypad 클릭시 실제 teleop-key에 맞게 ROS /cmd_vel 토픽으로 발행
- gazebo상에서 이 keypad 원격조종을 통해 움직이는 것까지 확인

#### 🔨 추가해야 할 점
- 현재 ros1, ros2에 따른 버전 다르게 제작중
- 재연결이나 끊김 문제가 있어서 개선 필요
- 영상 지연시간이 느릴때는 400정도 나옴. 개선 필요! -> 휴대폰, 컴퓨터로 테스트해서 그런 것이였음
  -> 결론: 10번정도 테스트해본 결과 최소: 100 | 평균: 180 | 최대: 200

---

## 📌 사용 방법
1. Signaling 서버 (`server.signaling`) 실행.
2. Python 송신기 (`sender_vanilla.py`) 실행. 
3. React 앱 (`receiver-app`) 실행.

---

## 💡 참고 사항
- `.env` 파일 설정 필요 (`receiver-app/.env`)
- 방화벽 설정 및 포트 확인 필수
