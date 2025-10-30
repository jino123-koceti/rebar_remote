# 철근 결속 로봇 제어 시스템 (Rebar Tying Robot Control System)

ROS2 Humble 기반 철근 결속 로봇 통합 제어 시스템

## 🤖 시스템 개요

이 프로젝트는 건설 현장에서 철근을 자동으로 결속하는 로봇의 제어 시스템입니다. ROS2 Humble을 기반으로 7개의 RMD 모터, Iron-MD 무선 리모콘, Seengrip 그리퍼, 리미트 센서 등을 통합 제어합니다.

### 주요 기능

- **7축 RMD 모터 제어**: 위치/속도 제어, 이벤트 기반 피드백
- **무선 리모콘 제어**: Iron-MD CAN 프로토콜 (4개 조이스틱, 24개 버튼)
- **듀얼 CAN 버스**: can2 (1Mbps, 모터), can3 (250kbps, 리모콘)
- **그리퍼 통합**: Seengrip Modbus RTU 제어
- **리미트 센서**: FASTECH EZI-IO Modbus TCP
- **헤드리스 운영**: 모니터 없는 환경에서 로그 중심 운영
- **자동 시퀀스**: S21/S22 작업 시퀀스 (하강→그리퍼 닫기, 트리거→그리퍼 열기→상승)

## 📋 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    Iron-MD Wireless Remote                  │
│              (CAN3 @ 250kbps - 4 Joysticks)                │
└──────────────────────────┬──────────────────────────────────┘
                           │
                    ┌──────▼──────┐
                    │  Ubuntu PC  │
                    │ ROS2 Humble │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐      ┌──────▼──────┐    ┌─────▼─────┐
   │  CAN2   │      │  Modbus RTU │    │Modbus TCP │
   │ 1 Mbps  │      │/dev/ttyUSB0 │    │192.168.0.3│
   └────┬────┘      └──────┬──────┘    └─────┬─────┘
        │                  │                  │
   ┌────▼────────────┐ ┌───▼────┐      ┌─────▼─────┐
   │ 7 RMD Motors    │ │Seengrip│      │ EZI-IO    │
   │ 0x141 - 0x147   │ │Gripper │      │  Sensors  │
   └─────────────────┘ └────────┘      └───────────┘
```

### 모터 구성

| Motor ID | Function | Control Type | Notes |
|----------|----------|--------------|-------|
| 0x141 | 좌측 주행 모터 | 속도 제어 (0xA2) | Differential drive |
| 0x142 | 우측 주행 모터 | 속도 제어 (0xA2) | Differential drive |
| 0x143 | 횡이동 (Lateral) | 위치 제어 (0xA4) | ±360° rotation |
| 0x144 | X축 스테이지 | 속도 제어 (0xA2) | Linear motion |
| 0x145 | Y축 스테이지 | 속도 제어 (0xA2) | Linear motion |
| 0x146 | Z축 (상하) | 위치 제어 (0xA4) | Work sequence |
| 0x147 | Yaw (회전) | 위치 제어 (0xA4) | ±30° rotation |

## 🚀 빠른 시작

### 필수 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Hardware**: 
  - CAN 인터페이스 2개 (can2, can3)
  - USB-Serial 변환기 (/dev/ttyUSB0)
  - Ethernet (192.168.0.3 네트워크)

### 설치

```bash
# 저장소 클론
git clone https://github.com/jino123-koceti/rebar_remote.git
cd rebar_remote

# ROS2 의존성 설치
sudo apt update
sudo apt install ros-humble-desktop python3-pip

# Python 패키지 설치
pip3 install python-can pymodbus pyserial

# CAN 인터페이스 설정
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can3 type can bitrate 250000
sudo ip link set can2 up
sudo ip link set can3 up

# 빌드
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 실행

#### 방법 1: 통합 스크립트 (권장)

```bash
# 헤드리스 환경 - 로그 중심 실행
./integrated_control_debug.sh

# 로그 실시간 모니터링
tail -f /var/log/robot_control/control_latest.log

# 로그 뷰어 사용
./view_logs.sh -f  # 실시간 로그
./view_logs.sh -e  # 에러만 보기
./view_logs.sh -j  # 조이스틱 로그
```

#### 방법 2: 개별 노드 실행

```bash
# 터미널 1: RMD 모터 제어
ros2 run rmd_robot_control position_control_node

# 터미널 2: EZI-IO 센서
ros2 run ezi_io_ros2 ezi_io_node --ros-args -p ip_address:=192.168.0.3

# 터미널 3: Seengrip 그리퍼
ros2 run seengrip_ros2 seengrip_node --ros-args -p serial_port:=/dev/ttyUSB0

# 터미널 4: Iron-MD 텔레옵
ros2 run rebar_control iron_md_teleop
```

## 🎮 리모콘 매핑 (Iron-MD)

### 조이스틱 (Analog)

| Joystick | Function | Motor | Range |
|----------|----------|-------|-------|
| AN3 | 전후진 | 0x141, 0x142 | -1.0 ~ 1.0 |
| AN4 | 좌우 회전 | 0x141, 0x142 | -1.0 ~ 1.0 |
| AN1 | X축 이동 | 0x144 | ±200 dps |
| AN2 | Y축 이동 | 0x145 | ±200 dps |

### 버튼 (Digital)

| Button | Function | Description |
|--------|----------|-------------|
| S13 | 브레이크 토글 | 모터 브레이크 해제/잠금 |
| S14 | 홈잉 | 드라이브 모터 홈 리미트 탐색 |
| S17 | 횡이동 + | 0x143 +360° 회전 |
| S18 | 횡이동 - | 0x143 -360° 회전 |
| S21 | 작업 시퀀스 1 | Z축 하강 → 그리퍼 닫기 |
| S22 | 작업 시퀀스 2 | 트리거 → 그리퍼 열기 → Z축 상승 |
| S23 | Yaw + | 0x147 +30° 회전 |
| S24 | Yaw - | 0x147 -30° 회전 |

## 📦 패키지 구조

### rebar_control
Iron-MD 무선 리모콘 CAN 통신 및 텔레옵 제어

**주요 파일:**
- `iron_md_teleop_node.py` (1,255줄): 메인 텔레옵 노드
- `iron_md_joystick.dbc`: CAN DBC 정의

**기능:**
- CAN 0x1E4 (조이스틱), 0x2E4 (스위치), 0x764 (heartbeat) 파싱
- ROS2 토픽 발행: `/cmd_vel`, `/joint_*/position`, `/gripper/position`
- 작업 시퀀스 타이머 기반 비동기 실행

### rmd_robot_control
RMD 모터 7개 통합 제어

**주요 파일:**
- `position_control_node.py` (1,140줄): 위치/속도 제어 노드
- `can_manager.py`: CAN 통신 매니저
- `rmd_x4_protocol.py`: RMD Protocol V4.3 구현

**기능:**
- 0xA4 절대 위치 제어 (0x143, 0x146, 0x147)
- 0xA2 속도 제어 (0x141, 0x142, 0x144, 0x145)
- 0x92 멀티턴 각도 읽기 (이벤트 기반)
- 모터 에러 상태 자동 감지 및 로깅

### seengrip_ros2
Seengrip 그리퍼 Modbus RTU 제어

**주요 파일:**
- `seengrip_node.py`: Modbus RTU 통신 노드

**기능:**
- `/gripper/position` 구독 (0.0=열림, 1.0=닫힘)
- Modbus 레지스터: 위치(40001), 속도(40002), 힘(40003)

### ezi_io_ros2
FASTECH EZI-IO 리미트 센서 Modbus TCP

**주요 파일:**
- `ezi_io_node.py`: Modbus TCP 통신 노드

**기능:**
- 6개 리미트 센서 모니터링 (IN00-IN06)
- ROS2 토픽 발행: `/limit_sensors/*`
- 10Hz 폴링

### pololu_ros2
Pololu Simple Motor Controller 제어

**주요 파일:**
- `pololu_node.py`: Pololu SMC 통신 노드
- `pololu_driver.py`: SMC 드라이버

**기능:**
- 트리거 모터 제어 (forward/reverse)
- USB 시리얼 통신

## 🔧 설정 파일

### CAN 인터페이스 자동 설정

```bash
# /etc/systemd/network/80-can.network
[Match]
Name=can2

[CAN]
BitRate=1M

[Match]
Name=can3

[CAN]
BitRate=250K
```

### 부팅 시 자동 실행

```bash
# /etc/systemd/system/robot-control.service
[Unit]
Description=Rebar Robot Control System
After=network.target

[Service]
Type=simple
User=test
WorkingDirectory=/home/test/ros2_ws
ExecStart=/home/test/ros2_ws/integrated_control_debug.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

활성화:
```bash
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service
```

## 📊 로그 관리

### 로그 위치

- **메인 로그**: `/var/log/robot_control/control_YYYYMMDD_HHMMSS.log`
- **최신 로그**: `/var/log/robot_control/control_latest.log` (심볼릭 링크)

### 로그 유틸리티

```bash
# 실시간 로그 보기
./view_logs.sh -f

# 에러만 필터링
./view_logs.sh -e

# CAN 메시지만 보기
./view_logs.sh -c

# 조이스틱 입력 확인
./view_logs.sh -j

# 모터 제어 로그
./view_logs.sh -m

# 로그 통계
./view_logs.sh -s

# 로그 정리 (7일 이상, 500MB 초과)
./cleanup_logs.sh
```

## 🐛 문제 해결

### CAN 버스 에러

```bash
# CAN 인터페이스 재시작
sudo ip link set can2 down
sudo ip link set can2 up

# CAN 에러 확인
ip -details -statistics link show can2

# Bus-off 복구
sudo ip link set can2 type can restart-ms 100
```

### 모터 에러 상태

모터 에러는 자동으로 로그에 기록됩니다:
```
[ERROR] MOTOR ERROR 0x146: Error State=0x04 (Overtemp=1)
```

에러 코드:
- `0x01`: Low Voltage (저전압)
- `0x02`: Overcurrent (과전류)
- `0x04`: Overtemp (과열)
- `0x08`: Encoder Error (엔코더 오류)
- `0x10`: Overload (과부하)

### 그리퍼 연결 실패

```bash
# 시리얼 포트 확인
ls -l /dev/ttyUSB*

# 권한 설정
sudo chmod 666 /dev/ttyUSB0

# Modbus RTU 통신 테스트
# (seengrip_ros2/seengrip_node.py 참고)
```

## 📚 문서

- [ROBOT_CONTROL_ARCHITECTURE.md](ROBOT_CONTROL_ARCHITECTURE.md): 전체 시스템 아키텍처
- [MOTOR_CONTROL_GUIDE.md](MOTOR_CONTROL_GUIDE.md): 모터 제어 상세 가이드
- [HEADLESS_OPERATION.md](HEADLESS_OPERATION.md): 헤드리스 환경 운영
- [FIX_MOTOR_CONTROL.md](FIX_MOTOR_CONTROL.md): 모터 제어 수정 사항
- [src/rebar_control/IRON_MD_GUIDE.md](src/rebar_control/IRON_MD_GUIDE.md): Iron-MD 리모콘 가이드

## 🔐 안전 기능

- **비상 정지**: Iron-MD 리모콘 비상정지 버튼 (즉시 모든 모터 정지)
- **리미트 센서**: EZI-IO 6개 센서로 동작 범위 제한
- **브레이크 시스템**: 전원 차단 시 자동 브레이크 작동
- **에러 감지**: 모터 과열/과전류 자동 감지 및 로깅
- **프로세스 모니터링**: 5분마다 노드 상태 확인 및 로깅

## 🤝 기여

버그 리포트, 기능 제안, Pull Request 환영합니다!

## 📝 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 👥 개발자

- **Koceti Robotics Team**
- GitHub: [@jino123-koceti](https://github.com/jino123-koceti)

## 📮 연락처

문제가 있거나 질문이 있으시면 GitHub Issues를 이용해주세요.

---

**Last Updated**: 2025-10-30  
**ROS2 Version**: Humble  
**Ubuntu Version**: 22.04 LTS
