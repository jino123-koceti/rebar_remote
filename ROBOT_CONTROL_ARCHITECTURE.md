# 철근 결속 로봇 제어 시스템 구조 분석

## 📋 목차
1. [시스템 개요](#시스템-개요)
2. [하드웨어 구성](#하드웨어-구성)
3. [ROS2 패키지 구조](#ros2-패키지-구조)
4. [통합 제어 스크립트](#통합-제어-스크립트)
5. [제어 흐름도](#제어-흐름도)
6. [ROS2 토픽 구조](#ros2-토픽-구조)
7. [노드별 상세 설명](#노드별-상세-설명)
8. [사용 방법](#사용-방법)

---

## 시스템 개요

철근 결속 로봇은 **무선 리모콘(Iron-MD)**으로 조작하여 다음 작업을 수행합니다:
- 주행 (전진/후진/회전)
- 3축 스테이지 제어 (X, Y, Z축)
- 리프팅 (상승/하강)
- 그리퍼 (열기/닫기)
- 결속 트리거 작동

### 주요 특징
- **2개 CAN 버스** 분리 운영 (모터 제어와 리모콘 통신 간섭 방지)
- **7개 RMD 모터** 위치/속도 제어
- **Pololu 리니어 액추에이터** 트리거 제어
- **Seengrip 그리퍼** Modbus RTU 제어
- **리모콘 입력 → ROS2 토픽 → 모터 제어** 구조

---

## 하드웨어 구성

### CAN 버스 구조

| CAN 인터페이스 | 용도 | 장치 | 통신 속도 |
|---------------|------|------|----------|
| **can2** | 모터 제어 | RMD X4/X8 모터 (0x141-0x147) | 1 Mbps |
| **can3** | 리모콘 통신 | Iron-MD 무선 조종기 | 250 kbps |

> **중요**: CAN 버스가 분리되어 있어 리모콘 통신과 모터 제어가 서로 간섭하지 않습니다.

### 모터 배치

#### CAN2 (모터 제어)
| 모터 ID | 기능 | 제어 방식 | 설명 |
|---------|------|----------|------|
| **0x141** | 좌측 구동륜 | 속도 제어 (cmd_vel) | 주행 전후진/회전 |
| **0x142** | 우측 구동륜 | 속도 제어 (cmd_vel) | 주행 전후진/회전 |
| **0x143** | 횡이동 | 위치 제어 | 좌우 이동 (50mm 단위) |
| **0x144** | X축 | 위치 제어 | 상부체 X축 이동 |
| **0x145** | Y축 | 위치 제어 | 상부체 Y축 이동 |
| **0x146** | Z축 (리프팅) | 위치 제어 | 상하 이동 |
| **0x147** | Yaw 회전 | 위치 제어 | 30도 단위 회전 |

#### 추가 디바이스
| 장치 | 포트 | 프로토콜 | 기능 |
|------|------|----------|------|
| **Pololu 18v7** | /dev/ttyACM0 | USB (SmcCmd) | 트리거 선형 액추에이터 |
| **Seengrip 그리퍼** | /dev/ttyUSB0 | Modbus RTU (115200) | 그립 열기/닫기 (0-2000) |
| **EZI-IO** | 192.168.0.2:502 | Modbus TCP | 리미트 센서 (옵션) |

---

## ROS2 패키지 구조

```
/home/test/ros2_ws/src/
├── rmd_robot_control/          # RMD 모터 제어 패키지
│   ├── position_control_node.py  # 통합 제어 노드 (7개 모터)
│   ├── can_manager.py            # CAN 버스 관리
│   ├── rmd_x4_protocol.py        # RMD 모터 프로토콜
│   └── launch/
│       └── robot_control.launch.py
│
├── rebar_control/              # 상위 통합 제어 패키지
│   ├── iron_md_teleop_node.py  # Iron-MD 리모콘 노드 (can3)
│   ├── teleop_node.py          # 일반 조이스틱 노드 (옵션)
│   ├── safety_monitor.py       # 안전 감시
│   └── launch/
│       ├── full_system.launch.py   # 전체 시스템 런치
│       ├── test_teleop.launch.py   # 리모콘 테스트
│       └── test_driving.launch.py  # 주행 테스트
│
├── pololu_ros2/                # Pololu 액추에이터 패키지
│   └── pololu_node.py          # 트리거 제어 노드
│
├── seengrip_ros2/              # Seengrip 그리퍼 패키지
│   └── seengrip_node.py        # 그리퍼 제어 노드
│
└── ezi_io_ros2/                # EZI-IO 리미트 센서 패키지
    └── ezi_io_node.py          # 센서 읽기 노드
```

---

## 통합 제어 스크립트

### `integrated_control.sh` 분석

```bash
#!/bin/bash
# 위치: /home/test/ros2_ws/integrated_control.sh

cd /home/test/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1단계: RMD 모터 제어 노드 실행 (can2)
ros2 run rmd_robot_control position_control_node &
MOTOR_PID=$!
sleep 3

# 2단계: Iron-MD 텔레옵 노드 실행 (can3)
ros2 run rebar_control iron_md_teleop &
TELEOP_PID=$!
sleep 2

# 계속 실행 (Ctrl+C로 종료)
wait
```

### 실행 순서
1. **position_control_node** 시작 → CAN2에서 7개 모터 제어 준비
2. 3초 대기 → 모터 노드 완전 초기화
3. **iron_md_teleop** 시작 → CAN3에서 리모콘 입력 수신
4. 2초 대기 → 리모콘 노드 초기화
5. 백그라운드 실행 유지

---

## 제어 흐름도

```
┌──────────────────────────────────────────────────────────────────┐
│                       Iron-MD 무선 리모콘                          │
│  (Joystick AN1/AN2/AN3, Switches S13/S14/S17-S24, Emergency Stop) │
└──────────────┬───────────────────────────────────────────────────┘
               │ CAN3 (250kbps)
               │ 0x1E4: 조이스틱 데이터 (50ms)
               │ 0x2E4: 스위치 상태 (50ms)
               ▼
┌──────────────────────────────────────────────────────────────────┐
│              iron_md_teleop_node (ROS2 노드)                      │
│  - CAN 메시지 파싱                                                │
│  - 조이스틱 → cmd_vel 변환                                        │
│  - 스위치 → 관절 위치 명령 변환                                   │
└──────────────┬───────────────────────────────────────────────────┘
               │ ROS2 토픽 발행
               │
               ├─► /cmd_vel (Twist)                  → 0x141, 0x142 (주행)
               ├─► /joint_1/position (Float64MultiArray) → 0x146 (리프팅)
               ├─► /joint_2/position (Float64MultiArray) → 0x144 (X축)
               ├─► /joint_3/position (Float64MultiArray) → 0x145 (Y축)
               ├─► /joint_4/position (Float64MultiArray) → 0x143 (횡이동)
               ├─► /joint_5/position (Float64MultiArray) → 0x147 (Yaw)
               ├─► /gripper/position (Float32)       → Seengrip
               └─► /motor_0/vel (Float32)            → Pololu 트리거
               
               ▼
┌──────────────────────────────────────────────────────────────────┐
│           position_control_node (통합 제어 노드)                   │
│  - cmd_vel → 차등 구동 계산 (좌우 바퀴 RPM)                       │
│  - 위치 명령 → RMD 위치 제어 명령 생성                            │
│  - 브레이크 서비스 제공 (safe_brake_release/lock)                │
└──────────────┬───────────────────────────────────────────────────┘
               │ CAN2 (1 Mbps)
               │
               ├─► 0x141 (좌측 바퀴): 속도 제어 명령
               ├─► 0x142 (우측 바퀴): 속도 제어 명령
               ├─► 0x143 (횡이동): 위치 제어 명령
               ├─► 0x144 (X축): 위치 제어 명령
               ├─► 0x145 (Y축): 위치 제어 명령
               ├─► 0x146 (Z축): 위치 제어 명령
               └─► 0x147 (Yaw): 위치 제어 명령
               
               ▼
┌──────────────────────────────────────────────────────────────────┐
│                      RMD X4/X8 모터 (7개)                         │
│  응답: 0x241-0x247 (위치, 속도, 토크 피드백)                      │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                    추가 디바이스 노드들                            │
│  - pololu_node: /dev/ttyACM0 (SmcCmd 래퍼)                        │
│  - seengrip_node: /dev/ttyUSB0 (Modbus RTU)                       │
│  - ezi_io_node: 192.168.0.2:502 (Modbus TCP, 옵션)               │
└──────────────────────────────────────────────────────────────────┘
```

---

## ROS2 토픽 구조

### 발행 토픽 (iron_md_teleop_node → position_control_node)

| 토픽 이름 | 메시지 타입 | 대상 모터 | 설명 |
|----------|-----------|----------|------|
| `/cmd_vel` | geometry_msgs/Twist | 0x141, 0x142 | 주행 제어 (linear.x, angular.z) |
| `/joint_1/position` | std_msgs/Float64MultiArray | 0x146 | 리프팅 위치 (도) |
| `/joint_2/position` | std_msgs/Float64MultiArray | 0x144 | X축 위치 (도) |
| `/joint_3/position` | std_msgs/Float64MultiArray | 0x145 | Y축 위치 (도) |
| `/joint_4/position` | std_msgs/Float64MultiArray | 0x143 | 횡이동 위치 (도) |
| `/joint_5/position` | std_msgs/Float64MultiArray | 0x147 | Yaw 회전 위치 (도) |
| `/gripper/position` | std_msgs/Float32 | Seengrip | 그리퍼 위치 (0-2000) |
| `/motor_0/vel` | std_msgs/Float32 | Pololu | 트리거 속도 (±3200) |
| `/emergency_stop` | std_msgs/Bool | 전체 | 비상 정지 플래그 |

### 구독 토픽 (position_control_node 피드백)

| 토픽 이름 | 메시지 타입 | 설명 |
|----------|-----------|------|
| `/joint_states` | sensor_msgs/JointState | 전체 관절 상태 (위치/속도/토크) |
| `/motor_status` | std_msgs/Float64MultiArray | 모터별 상태 배열 |
| `/motor_0x141_rpm` | std_msgs/Float32 | 좌측 바퀴 RPM |
| `/motor_0x142_rpm` | std_msgs/Float32 | 우측 바퀴 RPM |

### 서비스

| 서비스 이름 | 타입 | 설명 |
|------------|------|------|
| `/safe_brake_release` | std_srvs/Trigger | 브레이크 해제 (7개 모터 순차) |
| `/safe_brake_lock` | std_srvs/Trigger | 브레이크 잠금 (7개 모터 순차) |

---

## 노드별 상세 설명

### 1. `iron_md_teleop_node` (리모콘 입력 노드)

**파일**: `/home/test/ros2_ws/src/rebar_control/rebar_control/iron_md_teleop_node.py`

#### 주요 기능
- CAN3에서 Iron-MD 리모콘 메시지 수신
- 조이스틱 아날로그 값 → ROS2 토픽 변환
- 스위치 입력 → 모터 제어 명령 변환
- 브레이크 제어 (S13 버튼)
- 위치 리셋 (S14 버튼)

#### CAN 메시지 처리
```python
# 0x1E4 (484): 조이스틱 아날로그 데이터
AN1 (byte 0): X축 조이스틱 (0-255, 중립 127)
AN2 (byte 1): Y축 조이스틱
AN3 (byte 2): 주행 조이스틱
AN4 (byte 3): 예비

# 0x2E4 (740): 스위치 상태
S00-S24: 각종 버튼/스위치 (0=OFF, 1=ON)
Emergency_Stop_Active: 비상 정지 활성화
TX_Connected: 리모콘 연결 상태
```

#### 조이스틱 매핑
| 입력 | 출력 토픽 | 대상 모터 | 설명 |
|------|----------|----------|------|
| AN3 (세로) | `/cmd_vel` linear.x | 0x141, 0x142 | 전진/후진 |
| AN3 (가로) | `/cmd_vel` angular.z | 0x141, 0x142 | 좌우 회전 |
| AN1 | `/joint_2/position` | 0x144 | X축 이동 |
| AN2 | `/joint_3/position` | 0x145 | Y축 이동 |

#### 스위치 매핑
| 스위치 | 동작 | 대상 | 설명 |
|--------|------|------|------|
| S13 | 브레이크 토글 | 전체 모터 | 해제↔잠금 |
| S14 | 위치 리셋 | 위치제어 모터 | 현재 위치 → 0° |
| S17/S18 | 횡이동 ±50mm | 0x143 | 좌우 이동 |
| S23/S24 | Yaw ±30° | 0x147 | 회전 |
| S21/S22 | 작업 시퀀스 | 여러 모터 | 자동 작업 |
| S00 | 트리거 당김 | Pololu | 결속 건 작동 |
| Emergency | 비상 정지 | 전체 | 모든 모터 정지 |

#### 제어 루프
```python
# 20Hz (0.05초마다 실행)
def control_loop(self):
    # 1. 조이스틱 값 읽기 (AN1/AN2/AN3)
    # 2. cmd_vel 계산 및 발행 (주행)
    # 3. 관절 위치 명령 계산 (X/Y축)
    # 4. 스위치 상태 확인 (엣지 감지)
    # 5. 명령 발행
```

---

### 2. `position_control_node` (통합 모터 제어 노드)

**파일**: `/home/test/ros2_ws/src/rmd_robot_control/rmd_robot_control/position_control_node.py`

#### 주요 기능
- **7개 RMD 모터** 통합 제어
  - 0x141, 0x142: 속도 제어 (주행)
  - 0x143-0x147: 위치 제어 (관절)
- cmd_vel → 차등 구동 (differential drive) 변환
- 위치 명령 → RMD 위치 제어 프로토콜 변환
- 브레이크 서비스 제공
- CAN2 버스 통신

#### cmd_vel 처리 (차등 구동)
```python
def cmd_vel_callback(self, msg):
    linear_vel = msg.linear.x   # m/s
    angular_vel = msg.angular.z # rad/s
    
    # 차등 구동 계산
    left_vel = (linear_vel - angular_vel * wheel_base / 2) / wheel_radius
    right_vel = (linear_vel + angular_vel * wheel_base / 2) / wheel_radius
    
    # rad/s → RPM 변환
    left_rpm = left_vel * 60 / (2 * π)
    right_rpm = right_vel * 60 / (2 * π)
    
    # CAN 명령 전송
    send_speed_command(0x141, left_rpm)
    send_speed_command(0x142, right_rpm)
```

#### 위치 제어 처리
```python
def single_joint_callback(self, msg, joint_idx):
    motor_id = self.motor_ids[joint_idx]  # 0x143-0x147
    target_position = msg.data[0]  # 도(degree)
    
    # 안전 제한
    target_position = clamp(target_position, min_position, max_position)
    
    # RMD 위치 제어 명령 생성
    command = protocol.create_position_command(target_position)
    
    # CAN 전송
    can_manager.send_frame(motor_id, command)
```

#### 브레이크 제어
```python
def safe_brake_release(self):
    """순차적 브레이크 해제 (CAN 버스 보호)"""
    for motor_id in [0x143, 0x144, 0x145, 0x146, 0x147]:
        send_brake_release_command(motor_id)
        time.sleep(0.1)  # 100ms 간격
```

---

### 3. `pololu_node` (트리거 액추에이터)

**파일**: `/home/test/ros2_ws/src/pololu_ros2/pololu_ros2/pololu_node.py`

#### 주요 기능
- Pololu 18v7 Simple Motor Controller 제어
- `/motor_0/vel` 토픽 구독 (속도 명령)
- SmcCmd subprocess 래퍼 사용
- 트리거 당김/해제 동작

#### 제어 방식
```python
# SmcCmd 명령어 사용 (USB libusb)
def set_speed(self, speed):
    """속도 설정 (-3200 ~ +3200)"""
    subprocess.run(['SmcCmd', '--speed', str(speed)])

# 트리거 동작
trigger_pull():   set_speed(3200)   # 전진
trigger_release(): set_speed(-3200) # 후진
stop():           set_speed(0)      # 정지
```

---

### 4. `seengrip_node` (그리퍼 제어)

**파일**: `/home/test/ros2_ws/src/seengrip_ros2/seengrip_ros2/seengrip_node.py`

#### 주요 기능
- Seengrip 그리퍼 Modbus RTU 제어
- `/gripper/position` 토픽 구독
- RS485-USB 어댑터 사용 (/dev/ttyUSB0)

#### 제어 방식
```python
# Modbus RTU 레지스터 쓰기
def set_gripper_position(self, position):
    """그리퍼 위치 설정 (0-2000)"""
    mb.WriteReg(slave_id=1, MB_GOAL_SPD, 500)  # 속도
    mb.WriteReg(slave_id=1, MB_GOAL_POS, position)  # 목표 위치

# 미리 정의된 동작
open_gripper():  set_position(0)     # 완전 열림
close_gripper(): set_position(2000)  # 완전 닫힘
```

---

## 사용 방법

### 1. 통합 제어 시작

```bash
cd /home/test/ros2_ws
./integrated_control.sh
```

**실행 결과**:
```
==========================================
통합 제어 시스템 시작
==========================================

CAN 인터페이스 상태:
can2: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT
can3: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT

==========================================
1. RMD 모터 제어 노드 실행 (can2)
==========================================
✅ CAN can2 연결 성공 (통합 노드)
✅ 통합 노드 초기화 완료 (7개 모터)

==========================================
2. Iron-MD 텔레옵 노드 실행 (can3)
==========================================
✅ CAN can3 연결 성공 (Iron-MD)
✅ 조종기 연결됨 (TX_Connected)

==========================================
✅ 통합 제어 시스템 시작 완료!
==========================================
```

---

### 2. 브레이크 제어

#### 리모콘으로 (S13 버튼)
```
S13 버튼 1회 누름 → 브레이크 해제 (모터 움직일 수 있음)
S13 버튼 2회 누름 → 브레이크 잠금 (모터 고정)
```

#### 서비스로 (별도 터미널)
```bash
# 브레이크 해제
ros2 service call /safe_brake_release std_srvs/srv/Trigger

# 브레이크 잠금
ros2 service call /safe_brake_lock std_srvs/srv/Trigger
```

---

### 3. 조작 방법

#### 주행 (AN3 조이스틱)
```
AN3 세로 → 전진/후진
AN3 가로 → 좌우 회전

예: 전진하면서 우회전
  - AN3 세로 위로 + AN3 가로 오른쪽
  → cmd_vel: linear.x=0.5, angular.z=0.3
  → 0x141 (좌): 빠르게, 0x142 (우): 느리게
```

#### 스테이지 제어 (AN1/AN2 조이스틱)
```
AN1 가로 → X축 좌우 이동 (0x144)
AN2 세로 → Y축 전후 이동 (0x145)

예: X축 오른쪽으로 10mm 이동
  - AN1 가로 오른쪽
  → /joint_2/position: [10.0]  # 도(degree) 단위
  → 0x144로 위치 제어 명령 전송
```

#### 횡이동 (S17/S18 스위치)
```
S17 버튼 → 오른쪽 50mm 이동 (0x143)
S18 버튼 → 왼쪽 50mm 이동 (0x143)

예: S17 누름 → current_lateral + 50mm
```

#### Yaw 회전 (S23/S24 스위치)
```
S23 버튼 → 시계방향 30° (0x147)
S24 버튼 → 반시계방향 30° (0x147)

예: S23 누름 → current_yaw + 30°
```

#### 그리퍼 (R1/R2 버튼)
```
R1 버튼 → 그리퍼 열기 (position=0)
R2 버튼 → 그리퍼 닫기 (position=2000)
```

#### 트리거 (S00 버튼)
```
S00 버튼 누름 → 트리거 당김 (0.5초)
  - Pololu 전진 (3200)
  - 0.5초 후 자동 정지
```

#### 비상 정지
```
Emergency_Stop 버튼 → 모든 모터 즉시 정지
  - /emergency_stop 토픽: True
  - 모든 제어 명령 무시
  - START 버튼으로 해제
```

---

### 4. 모니터링 (별도 터미널)

#### ROS2 토픽 확인
```bash
# 주행 속도 확인
ros2 topic echo /cmd_vel

# 관절 상태 확인
ros2 topic echo /joint_states

# 개별 모터 RPM 확인
ros2 topic echo /motor_0x141_rpm
ros2 topic echo /motor_0x142_rpm
```

#### CAN 버스 모니터링
```bash
# 모터 제어 CAN 버스
candump can2

# 리모콘 CAN 버스
candump can3
```

#### 노드 상태 확인
```bash
# 실행 중인 노드 목록
ros2 node list

# 서비스 목록
ros2 service list

# 토픽 목록
ros2 topic list
```

---

### 5. 개별 테스트

#### 주행만 테스트
```bash
cd /home/test/ros2_ws
source install/setup.bash
ros2 run rmd_robot_control position_control_node &
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10
```

#### 위치 제어만 테스트
```bash
# X축 모터 (0x144) 10도 이동
ros2 topic pub /joint_2/position std_msgs/Float64MultiArray "{data: [10.0]}" --once
```

#### 그리퍼 테스트
```bash
# 그리퍼 닫기
ros2 topic pub /gripper/position std_msgs/Float32 "{data: 2000.0}" --once

# 그리퍼 열기
ros2 topic pub /gripper/position std_msgs/Float32 "{data: 0.0}" --once
```

#### Pololu 트리거 테스트
```bash
# 전진 (당김)
ros2 topic pub /motor_0/vel std_msgs/Float32 "{data: 3200.0}" --once

# 정지
ros2 topic pub /motor_0/vel std_msgs/Float32 "{data: 0.0}" --once

# 후진 (해제)
ros2 topic pub /motor_0/vel std_msgs/Float32 "{data: -3200.0}" --once
```

---

## 트러블슈팅

### CAN 버스 오류

#### Bus-off 상태
```bash
# 증상: 모터가 응답하지 않음
# 원인: CAN 버스 과부하, 동시 전송 과다

# 해결:
sudo ip link set can2 type can restart-ms 100
sudo ip link set can2 up
```

#### CAN 인터페이스 DOWN
```bash
# 확인
ip link show can2
ip link show can3

# 재시작
sudo ip link set can2 down
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 down
sudo ip link set can3 up type can bitrate 250000
```

### 브레이크 문제

#### 브레이크가 해제되지 않음
```bash
# 서비스로 강제 해제
ros2 service call /safe_brake_release std_srvs/srv/Trigger

# 개별 모터 해제 (candump으로 확인)
candump can2 &
ros2 service call /safe_brake_release std_srvs/srv/Trigger
```

### 리모콘 연결 문제

#### TX_Connected = 0 (연결 안 됨)
```bash
# CAN3 상태 확인
ip link show can3

# Heartbeat 확인 (1892, 0x764)
candump can3 | grep 764

# 리모콘 전원 확인 및 재부팅
```

### Pololu/Seengrip 문제

#### Pololu 응답 없음
```bash
# 포트 확인
ls -l /dev/ttyACM*

# SmcCmd 테스트
SmcCmd --status

# 수동 제어
SmcCmd --speed 1000
SmcCmd --stop
```

#### Seengrip 응답 없음
```bash
# 포트 확인
ls -l /dev/ttyUSB*

# 테스트 스크립트 실행
cd /home/test/ros2_ws
python3 test_seengrip_simplemodbus.py
```

---

## 요약

### 제어 흐름
```
리모콘 입력 (CAN3)
  ↓
iron_md_teleop_node (CAN 파싱)
  ↓
ROS2 토픽 (/cmd_vel, /joint_X/position)
  ↓
position_control_node (모터 명령 생성)
  ↓
CAN2 전송 → RMD 모터 (0x141-0x147)
```

### 핵심 구성요소
- **2개 CAN 버스**: can2(모터), can3(리모콘)
- **2개 ROS2 노드**: iron_md_teleop, position_control_node
- **7개 RMD 모터**: 주행(2) + 관절(5)
- **2개 추가 디바이스**: Pololu, Seengrip

### 주요 토픽
- `/cmd_vel`: 주행 제어 (0x141, 0x142)
- `/joint_X/position`: 관절 위치 제어 (0x143-0x147)
- `/gripper/position`: 그리퍼 위치
- `/motor_0/vel`: 트리거 속도

### 안전 기능
- S13: 브레이크 토글
- Emergency_Stop: 비상 정지
- 순차 브레이크 해제 (CAN 버스 보호)
- 위치 제한 (min/max clamp)

---

## 참고 문서
- `MOTOR_CONTROL_GUIDE.md` - Pololu/Seengrip 개별 제어
- `CAN_REMOTE_CONTROL_적용완료.md` - CAN 리모콘 설정
- `MOTOR_CONTROL_GUIDE.md` - 모터 제어 가이드
