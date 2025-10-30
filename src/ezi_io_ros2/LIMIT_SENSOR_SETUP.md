# 상부체 스테이지 리미트 센서 설정 완료 ✅

## 📊 리미트 센서 매핑 (실제 배선)

### EZI-IO-EN-L16O16N-T (192.168.0.3)

| 입력 핀 | 축 | 설명 | ROS2 토픽 |
|---------|-----|------|-----------|
| **IN00** | Y축 | 최대 리미트 | `/limit_sensors/y_max` |
| **IN01** | Y축 | 원점 리미트 | `/limit_sensors/y_min` |
| **IN02** | X축 | 원점 리미트 | `/limit_sensors/x_min` |
| **IN03** | X축 | 최대 리미트 | `/limit_sensors/x_max` |
| **IN04** | Yaw | 원점 리미트 | `/limit_sensors/yaw_min` |
| **IN05** | Z축 | 원점 리미트 | `/limit_sensors/z_min` |
| **IN06** | Z축 | 최대 리미트 | `/limit_sensors/z_max` |

## 🔧 변경 사항

### 1. **설정 파일 업데이트** (`config/ezi_io.yaml`)
```yaml
ip_address: "192.168.0.3"  # 실제 EZI-IO IP

# 실제 배선에 맞춘 채널 매핑
limit_x_min_channel: 2  # IN02
limit_x_max_channel: 3  # IN03
limit_y_min_channel: 1  # IN01
limit_y_max_channel: 0  # IN00
limit_z_min_channel: 5  # IN05
limit_z_max_channel: 6  # IN06
limit_yaw_min_channel: 4  # IN04
```

### 2. **노드 코드 업데이트** (`ezi_io_node.py`)
- Yaw축 원점 리미트 센서 추가
- 기본 채널 매핑을 실제 배선에 맞춤
- `/limit_sensors/yaw_min` 토픽 추가

### 3. **README 업데이트**
- IP 주소: 192.168.0.3으로 변경
- 실제 배선 매핑 표 추가
- Yaw축 센서 설명 추가

### 4. **테스트 스크립트 생성** (`test_limit_sensors.py`)
- FASTECH Plus-E 라이브러리 사용
- 실시간 센서 상태 모니터링
- 트리거된 센서 경고 표시

## 🚀 사용법

### 1. 하드웨어 테스트
```bash
# EZI-IO 직접 테스트 (Plus-E 라이브러리)
cd ~/ros2_ws
python3 test_limit_sensors.py

# 예상 출력:
# IN00: 🟢 정상     (Y축 최대)
# IN01: 🟢 정상     (Y축 원점)
# IN02: 🔴 트리거됨 (X축 원점)  ← 원점 위치일 때
# ...
```

### 2. ROS2 노드 실행
```bash
# 빌드
cd ~/ros2_ws
colcon build --packages-select ezi_io_ros2
source install/setup.bash

# 노드 실행
ros2 launch ezi_io_ros2 ezi_io.launch.py

# 또는 직접 실행
ros2 run ezi_io_ros2 ezi_io_node
```

### 3. 리미트 센서 모니터링
```bash
# 모든 센서 상태 확인
ros2 topic echo /ezi_io/diagnostics

# 개별 센서 확인
ros2 topic echo /limit_sensors/x_min
ros2 topic echo /limit_sensors/x_max
ros2 topic echo /limit_sensors/y_min
ros2 topic echo /limit_sensors/y_max
ros2 topic echo /limit_sensors/z_min
ros2 topic echo /limit_sensors/z_max
ros2 topic echo /limit_sensors/yaw_min

# 센서 변화 감지 (Hz 확인)
ros2 topic hz /limit_sensors/x_min
```

### 4. 다른 노드에서 사용
```bash
# 예: rmd_robot_control GUI에서 리미트 센서 구독
# (통합 예정)
```

## 📡 ROS2 토픽 상세

### Published Topics
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/limit_sensors/x_min` | std_msgs/Bool | 20Hz | X축 원점 (IN02) - True=트리거 |
| `/limit_sensors/x_max` | std_msgs/Bool | 20Hz | X축 최대 (IN03) - True=트리거 |
| `/limit_sensors/y_min` | std_msgs/Bool | 20Hz | Y축 원점 (IN01) - True=트리거 |
| `/limit_sensors/y_max` | std_msgs/Bool | 20Hz | Y축 최대 (IN00) - True=트리거 |
| `/limit_sensors/z_min` | std_msgs/Bool | 20Hz | Z축 원점 (IN05) - True=트리거 |
| `/limit_sensors/z_max` | std_msgs/Bool | 20Hz | Z축 최대 (IN06) - True=트리거 |
| `/limit_sensors/yaw_min` | std_msgs/Bool | 20Hz | Yaw 원점 (IN04) - True=트리거 |
| `/ezi_io/diagnostics` | diagnostic_msgs/DiagnosticArray | 20Hz | 전체 상태 |

**센서 상태:**
- `True` = 리미트 트리거됨 (센서 활성화, 위험!)
- `False` = 정상 (안전 범위)

## 🛡️ 안전 통합 (다음 단계)

### RMD Robot Control GUI 통합
```python
# robot_control_gui.py에 추가 예정:

class RobotControlGUI:
    def __init__(self):
        # 리미트 센서 구독
        self.limit_subscriptions = {}
        limits = ['x_min', 'x_max', 'y_min', 'y_max',
                  'z_min', 'z_max', 'yaw_min']

        for limit in limits:
            self.limit_subscriptions[limit] = self.ros_node.create_subscription(
                Bool, f'/limit_sensors/{limit}',
                lambda msg, l=limit: self.limit_callback(l, msg), 10
            )

    def limit_callback(self, limit_name, msg):
        """리미트 센서 콜백"""
        if msg.data:  # 트리거됨
            self.log_message(f"⚠️ {limit_name} 리미트 트리거!")
            # GUI에 빨간색 경고 표시
            # 해당 축 모터 정지
```

### Position Control Node 통합
```python
# position_control_node.py에 추가 예정:

class PositionControlNode:
    def __init__(self):
        # 리미트 상태 저장
        self.limits = {
            'x_min': False, 'x_max': False,
            'y_min': False, 'y_max': False,
            'z_min': False, 'z_max': False,
            'yaw_min': False,
        }

        # 구독
        for name in self.limits.keys():
            self.create_subscription(
                Bool, f'/limit_sensors/{name}',
                lambda msg, n=name: self.update_limit(n, msg), 10
            )

    def send_position_command(self, motor_id, target_position):
        """위치 명령 전송 전 리미트 체크"""
        # X축 (0x144)
        if motor_id == 0x144:
            current = self.motor_states[motor_id]['position']
            if target_position > current and self.limits['x_max']:
                self.get_logger().warn("❌ X축 최대 리미트! 명령 거부")
                return
            if target_position < current and self.limits['x_min']:
                self.get_logger().warn("❌ X축 원점 리미트! 명령 거부")
                return

        # ... Y, Z, Yaw 동일

        # 안전하면 명령 전송
        self.can_manager.send_frame(motor_id, command_data)
```

## 🔍 트러블슈팅

### 1. 연결 안됨
```bash
# Ping 테스트
ping 192.168.0.3

# 네트워크 확인
ip addr show
ip route

# 수동 IP 설정 (필요 시)
sudo ip addr add 192.168.0.100/24 dev eth0
```

### 2. 센서 값 안 읽힘
```bash
# Modbus 연결 확인
python3 test_ezi_io.py

# 노드 로그 확인
ros2 run ezi_io_ros2 ezi_io_node

# 예상 로그:
# [INFO] Connected to EZI-IO at 192.168.0.3:502
# [INFO] Limit x_min: CLEAR
# [INFO] Limit x_min: TRIGGERED  ← 센서 트리거 시
```

### 3. 잘못된 센서 매핑
```bash
# 테스트 스크립트로 실제 핀 확인
python3 test_limit_sensors.py

# X축을 수동으로 원점까지 이동
# -> IN02가 트리거되는지 확인
# -> 안되면 설정 파일 수정
```

### 4. pymodbus 없음
```bash
pip3 install pymodbus
```

## 📝 다음 단계

- [ ] RMD Robot Control GUI에 리미트 센서 상태 표시 추가
- [ ] Position Control Node에 리미트 체크 로직 추가
- [ ] 리미트 도달 시 자동 모터 정지
- [ ] 역방향 이동만 허용하는 안전 로직
- [ ] 리미트 센서 히스토리 로깅
- [ ] GUI에서 리미트 센서 테스트 버튼 추가

## ✅ 완료 체크리스트

- [x] EZI-IO 설정 파일 업데이트 (IP, 채널 매핑)
- [x] ezi_io_node.py Yaw 센서 추가
- [x] README.md 실제 배선 정보 반영
- [x] test_limit_sensors.py 테스트 스크립트 작성
- [ ] ROS2 빌드 및 테스트
- [ ] 실제 하드웨어 검증
- [ ] GUI 통합
- [ ] 안전 로직 통합

---

**작성일**: 2025-10-29
**상태**: 설정 완료 (테스트 대기)
