# 철근 결속 자동화 로봇 - 통합 하위제어 시스템

ROS2 Humble 기반 철근 결속 로봇의 완전한 하위제어 시스템입니다.

## 📋 시스템 개요

### 로봇 구성

```
┌─────────────────────────────────────────────────────────┐
│              철근 결속 자동화 로봇                       │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  [하부체 - 주행 시스템]                                   │
│   ├─ 0x141, 0x142: 무한궤도 주행 모터 (RMD-X4)          │
│   └─ 0x143: 리프팅 조인트 (50mm step 상승/하강)          │
│                                                           │
│  [상부체 - 작업 시스템]                                   │
│   ├─ XYZ 스테이지 (공구 이송)                            │
│   │   ├─ 0x144: X축 모터                                 │
│   │   ├─ 0x145: Y축 모터                                 │
│   │   └─ 0x146: Z축 모터                                 │
│   ├─ 0x147: Yaw 회전 모터                                │
│   ├─ Pololu 액추에이터: 결속건 트리거                     │
│   └─ Seengrip 그리퍼: 철근 고정                          │
│                                                           │
│  [센서]                                                   │
│   ├─ EZI-IO: XYZ 스테이지 리미트 센서 (6개)              │
│   ├─ ZED X (2대): 전/후방 주행용                         │
│   └─ ZED X Mini (2대): 작업영역 교차점 식별              │
│                                                           │
│  [제어]                                                   │
│   └─ Iron-MD 무선 조종기: CAN 통신 단동 제어             │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

### 작업 영역
- 1회 이동당: 2x3 철근 결속 구역 (6개 결속점)
- 각 구역에서 순차적으로 결속 작업 수행

## 🔧 하드웨어 구성

| 컴포넌트 | 통신 방식 | 포트/인터페이스 | 패키지 |
|----------|-----------|-----------------|--------|
| RMD-X4 모터 (7개) | CAN | can0 | rmd_robot_control |
| Pololu 액추에이터 | USB Serial | /dev/ttyACM0 | pololu_ros2 |
| Seengrip 그리퍼 | RS485 (Modbus RTU) | /dev/ttyUSB0 | seengrip_ros2 |
| EZI-IO 모듈 | Ethernet (Modbus TCP) | 192.168.0.2:502 | ezi_io_ros2 |
| ZED X (2대) | USB 3.0 | - | zed_wrapper |
| ZED X Mini (2대) | USB 3.0 | - | zed_wrapper |
| Iron-MD 조종기 | **CAN 250Kbps** | **can1** | rebar_control |

## 📦 ROS2 패키지 구조

```
src/
├── rmd_robot_control/      # RMD-X4 모터 제어 (0x141-0x147)
├── pololu_ros2/            # Pololu 액추에이터 제어
├── seengrip_ros2/          # Seengrip 그리퍼 제어
├── ezi_io_ros2/            # EZI-IO 리미트 센서
├── rebar_control/          # 통합 제어 및 텔레옵
└── zed_wrapper/            # ZED 카메라 (Stereolabs 공식)
```

## 🚀 설치 및 빌드

### 의존성 설치

```bash
# ROS2 Humble 설치 필요
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-joy \
    python3-serial \
    python3-pip

# Python 패키지
pip3 install pymodbus
```

### 빌드

```bash
cd ~/ros2_ws_backup
colcon build
source install/setup.bash
```

## 🎮 사용법

### 1. 전체 시스템 실행

```bash
# 모든 하드웨어 노드 + 텔레옵
ros2 launch rebar_control full_system.launch.py
```

### 2. 개별 테스트

#### 주행 시스템만 테스트
```bash
ros2 launch rebar_control test_driving.launch.py
```

#### XYZ 스테이지 + 리미트 센서 테스트
```bash
ros2 launch rebar_control test_stage.launch.py
```

#### 그리퍼 + 트리거 테스트
```bash
ros2 launch rebar_control test_gripper.launch.py
```

## 🎮 Iron-MD 조종기 매핑

### 연속 제어 (아날로그 조이스틱)
- **AN3 (조이스틱 3)**: 하부체 전후진 (0x141, 0x142)
- **AN1 (조이스틱 1)**: 상부체 X축 전후 (0x144)
- **AN2 (조이스틱 2)**: 상부체 Y축 전후 (0x145)

### 모드 선택 (3단 토글 스위치)
- **S19 (상단)**: Remote Control 모드 (조종기 제어)
- **중립**: 대기
- **S20 (하단)**: Automatic Control 모드 (상위제어 연동)

### 횡이동 (3단 토글 스위치 - 엣지 트리거)
- **S17 ON→OFF**: 하부체 횡이동 +50mm (0x143, 360도 회전)
- **S18 ON→OFF**: 하부체 횡이동 -50mm (0x143, -360도 회전)

### 작업 시퀀스 (3단 토글 스위치 - 엣지 트리거)
- **S21 ON→OFF**: Z축 100mm 하강 → 그리퍼 닫기 (간격 0)
- **S22 ON→OFF**: 트리거 동작 → 그리퍼 열기 (간격 2000) → Z축 100mm 상승

### Yaw 회전 (3단 토글 스위치 - 엣지 트리거)
- **S23 ON→OFF**: 공구 Yaw +30도 회전 (0x147)
- **S24 ON→OFF**: 공구 Yaw -30도 회전 (0x147)

### 기타 제어
- **S00**: 트리거 당김 (수동)
- **S01**: 트리거 해제 (수동)
- **Emergency Stop**: 비상 정지

> **참고**: S17-S18, S21-S22, S23-S24는 3단 스위치로 ON 신호 후 중립으로 복귀하여 1회만 동작합니다.

## 📡 주요 토픽

### 명령 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 주행 제어 |
| `/joint_1/position` | Float64MultiArray | 리프팅 위치 |
| `/joint_2/position` | Float64MultiArray | X축 위치 |
| `/joint_3/position` | Float64MultiArray | Y축 위치 |
| `/joint_4/position` | Float64MultiArray | Z축 위치 |
| `/joint_5/position` | Float64MultiArray | Yaw 위치 |
| `/motor_0/vel` | std_msgs/Float32 | 트리거 속도 |
| `/gripper/position` | std_msgs/Float32 | 그리퍼 위치 |
| `/emergency_stop` | std_msgs/Bool | 비상 정지 |

### 상태 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 전체 관절 상태 |
| `/limit_sensors/x_min` | std_msgs/Bool | X축 최소 리미트 |
| `/limit_sensors/x_max` | std_msgs/Bool | X축 최대 리미트 |
| `/limit_sensors/y_min` | std_msgs/Bool | Y축 최소 리미트 |
| `/limit_sensors/y_max` | std_msgs/Bool | Y축 최대 리미트 |
| `/limit_sensors/z_min` | std_msgs/Bool | Z축 최소 리미트 |
| `/limit_sensors/z_max` | std_msgs/Bool | Z축 최대 리미트 |
| `/system/safe` | std_msgs/Bool | 시스템 안전 상태 |

## 🛡️ 안전 기능

### 리미트 센서 모니터링
- XYZ 스테이지 각 축의 min/max 리미트 센서 실시간 감시
- 리미트 트리거 시 해당 방향 이동 차단
- 경고 메시지 출력

### 비상 정지
- SELECT 버튼으로 즉시 모든 모터 정지
- 비상 정지 상태에서는 모든 제어 명령 무시
- START 버튼으로 해제

### 안전 모니터
- `safety_monitor` 노드가 모든 리미트 센서 감시
- `/system/safe` 토픽으로 안전 상태 발행
- 위험 상황 자동 감지 및 경고

## 📝 작업 시퀀스 예시

```python
# 1. 주행: 작업 위치로 이동
#    - 무한궤도로 전진/후진, 회전
#    - ZED X 카메라로 장애물 회피

# 2. 리프팅: 높이 조정 (step by step)
#    - 0x143 모터로 50mm씩 상승
#    - 작업 높이에 도달

# 3. 비전: 교차점 식별
#    - ZED X Mini로 철근 교차점 탐지
#    - 2x3 그리드에서 작업 순서 결정

# 4. XYZ 스테이지: 공구 이송
#    - 0x144 (X), 0x145 (Y), 0x146 (Z)로 정밀 위치 이동
#    - 0x147 (Yaw)로 각도 조정

# 5. 그리퍼: 철근 고정
#    - Seengrip으로 교차점 철근 파지
#    - 흔들림 방지

# 6. 결속: 트리거 동작
#    - Pololu 액추에이터로 트리거 당김
#    - 결속선 감김 + 절단 (자동)

# 7. 다음 포인트: 반복
#    - 2x3 구역 내 6개 포인트 순차 작업
```

## 🔍 디버깅 및 모니터링

### 토픽 확인
```bash
# 조이스틱 입력 확인
ros2 topic echo /joy

# 주행 명령 확인
ros2 topic echo /cmd_vel

# 리미트 센서 상태
ros2 topic echo /limit_sensors/x_min

# 관절 상태
ros2 topic echo /joint_states
```

### 노드 상태
```bash
# 실행 중인 노드 확인
ros2 node list

# 특정 노드 정보
ros2 node info /teleop
```

### CAN 통신 확인
```bash
# CAN 인터페이스 상태
ip link show can0  # RMD 모터
ip link show can1  # Iron-MD 조종기

# CAN 메시지 모니터
candump can0  # RMD 모터 메시지
candump can1  # Iron-MD 조종기 메시지

# Iron-MD 조이스틱 데이터 확인
candump can1,1E4:7FF

# Iron-MD 스위치 상태 확인
candump can1,2E4:7FF
```

## 📚 개별 패키지 문서

각 패키지의 자세한 사용법은 해당 README 참고:

- [rmd_robot_control/README.md](src/rmd_robot_control/README.md)
- [pololu_ros2/README.md](src/pololu_ros2/README.md)
- [seengrip_ros2/README.md](src/seengrip_ros2/README.md)
- [ezi_io_ros2/README.md](src/ezi_io_ros2/README.md)

## 🔧 트러블슈팅

### CAN 통신 문제
```bash
# CAN 인터페이스 활성화
sudo ip link set can0 up type can bitrate 1000000
```

### 시리얼 포트 권한
```bash
# 임시 권한
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0

# 영구 권한 (재로그인 필요)
sudo usermod -a -G dialout $USER
```

### Iron-MD 조종기 인식 안됨
```bash
# CAN 인터페이스 확인
ip link show can1

# CAN1 활성화 (250 Kbps)
sudo ip link set can1 up type can bitrate 250000

# 메시지 수신 테스트
candump can1

# 예상 출력:
# can1  1E4   [8]  7F 7F 7F 7F 00 00 00 00  <- 조이스틱
# can1  2E4   [8]  00 00 00 00 00 00 00 00  <- 스위치
# can1  764   [8]  00 00 00 00 00 00 00 00  <- Heartbeat
```

### EZI-IO 연결 안됨
```bash
# IP 연결 확인
ping 192.168.0.2

# 네트워크 인터페이스 설정
sudo ip addr add 192.168.0.100/24 dev eth0
```

## 🚦 다음 단계

이 하위제어 시스템은 **무선 조종기를 통한 단동 제어**를 완벽하게 지원합니다.

향후 개발:
1. **상위 제어 시스템**: 자동 작업 시퀀스
2. **비전 통합**: ZED 카메라 데이터 활용
3. **경로 계획**: 자동 네비게이션
4. **작업 관리**: 2x3 그리드 작업 스케줄링

## 📄 라이센스

MIT License

## 👥 기여

본 시스템은 철근 결속 자동화를 위한 하위 제어 패키지입니다.
상위 제어 시스템과 통합하여 완전한 자동화를 구현할 수 있습니다.
