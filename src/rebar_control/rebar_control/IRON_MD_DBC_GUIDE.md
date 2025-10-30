# IRON-MD 조이스틱 CAN DBC 파일 가이드

## 📄 파일 정보
- **파일명**: `iron_md_joystick.dbc`
- **장치**: IRON-MD 무선 원격 조이스틱 (DAS_99_MD)
- **통신**: CAN 2.0B
- **Baudrate**: 125/250/500 Kbps (선택 가능, 기본 250Kbps)

## 🔌 하드웨어 연결
### IRON-MD 수신기 7핀 커넥터 (SF1212/S7)
```
1: CAN_L (흰색)
2: CAN_H (녹색)
3: NC
4: Power GND (0V) (흑색)
5: Power+ (24V) (빨강)
6: RS232_TX (청색)
7: RS232_RX (보라)
```

### IRON-MD 송신기측 7핀 커넥터
```
1: USB-5V_in (빨강)
2: USB-DP (노랑)
3: CAN-H/RS232 TX (청색)
4: POWER 24V_in (빨강)
5: CAN-L/RS232 RX (보라)
6: USB-DN (백색)
7: 0V/GND (흑색/초록)
```

## 📡 CAN 메시지 정의

### 1️⃣ 조이스틱 데이터 (0x1E4 / 484)
- **방향**: TX (송신기 → 수신기)
- **주기**: 50ms
- **길이**: 8 bytes

| Byte | Signal | 범위 | 설명 |
|------|--------|------|------|
| 0 | Joystick_1 | 0-255 | 조이스틱 1 아날로그 값 |
| 1 | Joystick_2 | 0-255 | 조이스틱 2 아날로그 값 |
| 2 | Joystick_3 | 0-255 | 조이스틱 3 아날로그 값 |
| 3 | Joystick_4 | 0-255 | 조이스틱 4 아날로그 값 |
| 4-7 | - | - | 사용 안함 |

### 2️⃣ 스위치 상태 (0x2E4 / 740)
- **방향**: TX (송신기 → 수신기)
- **주기**: 50ms
- **길이**: 8 bytes

#### Byte 0 (제어 버튼)
| Bit | Signal | 설명 |
|-----|--------|------|
| 0 | Start_Key | 시작 키 |
| 1 | Power_Key | 전원 온 키 |
| 2 | Engine_Start | 엔진 시작 (S13) |
| 3 | Engine_Stop | 엔진 정지 (S14) |
| 4-5 | - | 사용 안함 |
| 6 | Emergency_Stop_Release | 0: 비상정지, 1: 해제 |
| 7 | Emergency_Stop_Active | 1: 비상정지, 0: 해제 |

#### Byte 1 (스위치 S00-S07)
| Bit | Signal | 설명 |
|-----|--------|------|
| 0 | Switch_S06 | 스위치 S06 |
| 1 | Switch_S07 | 스위치 S07 |
| 2-3 | - | 사용 안함 |
| 4 | Switch_S02 | 스위치 S02 |
| 5 | Switch_S03 | 스위치 S03 |
| 6 | Switch_S01 | 스위치 S01 |
| 7 | Switch_S00 | 스위치 S00 |

#### Byte 2 (스위치 S08-S09)
| Bit | Signal | 설명 |
|-----|--------|------|
| 0 | Switch_S08 | 스위치 S08 |
| 1 | Switch_S09 | 스위치 S09 |
| 2-7 | - | 사용 안함 |

#### Byte 3 (스위치 S17-S24)
| Bit | Signal | 설명 |
|-----|--------|------|
| 0 | Switch_S23 | 스위치 S23 |
| 1 | Switch_S24 | 스위치 S24 |
| 2 | Switch_S21 | 스위치 S21 |
| 3 | Switch_S22 | 스위치 S22 |
| 4 | Switch_S19 | 스위치 S19 |
| 5 | Switch_S20 | 스위치 S20 |
| 6 | Switch_S17 | 스위치 S17 |
| 7 | Switch_S18 | 스위치 S18 |

#### Byte 4 (상태 정보)
| Bit | Signal | 범위 | 설명 |
|-----|--------|------|------|
| 0-3 | Run_Counter | 0-15 | 수신 신호 카운터 |
| 4 | Cal_Mode | 0/1 | 0: 일반모드, 1: 조정모드 |
| 5-7 | - | - | 사용 안함 |

#### Byte 5-7 (장치 정보)
| Byte | Bit | Signal | 범위 | 설명 |
|------|-----|--------|------|------|
| 5 | 6 | TX_Connected | 0/1 | 1: 송신기 연결됨 |
| 5 | 7 | TRX_Code | 0/1 | 1: 송신기, 0: 수신기 |
| 6 | 0-7 | Serial_Number | 0-255 | 기기 번호 (참고용) |
| 7 | 0-7 | Device_ID | 0-255 | 기기 ID (참고용) |

### 3️⃣ Heartbeat (0x764 / 1892)
- **방향**: TX (송신기 → 수신기)
- **주기**: 300ms
- **길이**: 8 bytes

| Byte | Signal | 값 | 설명 |
|------|--------|-----|------|
| 0 | Heart_Bit | 0 | 생존 신호, 항상 0 출력 |

### 4️⃣ LCD 디스플레이 (0x364 / 868) - 옵션, 미사용
- **방향**: RX (수신기 → 송신기)
- **주기**: >100ms
- **길이**: 8 bytes

| Byte | Signal | 범위 | 설명 |
|------|--------|------|------|
| 0 | Line_Index | 0-15 | LCD 라인 번호 (12 이상 무시) |
| 1 | Data_Index | 0-10 | 문자 위치 (8 이상 무시) |
| 2-7 | Data_0-5 | 0-255 | 표시할 문자 데이터 |

## 🚀 사용 방법

### 1. CAN 인터페이스 설정
```bash
# CAN0을 250Kbps로 설정
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

# 다른 Baudrate 사용시
# 125Kbps: bitrate 125000
# 500Kbps: bitrate 500000
```

### 2. CAN 메시지 모니터링
```bash
# 편리한 스크립트 사용
./can_monitor.sh

# 또는 직접 candump 사용
candump can0

# 특정 ID만 필터링
candump can0,1E4:7FF  # 조이스틱 데이터만
candump can0,2E4:7FF  # 스위치 상태만
```

### 3. ROS2에서 사용
```bash
# ros2_socketcan 사용 예시
ros2 run socketcan_bridge socketcan_bridge_node --ros-args \
  -p interface:=can0 \
  -p dbc_file:=/home/koceti/ros2_ws/iron_md_joystick.dbc
```

## 📊 데이터 해석 예시

### 조이스틱 중앙값
```
CAN ID: 0x1E4
Data: 80 80 80 80 00 00 00 00
해석: 모든 조이스틱이 중앙 (128, 0x80)
```

### 비상정지 활성화
```
CAN ID: 0x2E4
Data: C0 00 00 00 ...
해석: Bit 7=1, Bit 6=0 → 비상정지 상태
```

### 엔진 시작 버튼
```
CAN ID: 0x2E4
Data: 04 00 00 00 ...
해석: Bit 2=1 → 엔진 시작 (S13)
```

## 🔍 트러블슈팅

### CAN 메시지가 안 보일 때
```bash
# 1. 인터페이스 상태 확인
ip link show can0
# UP,RUNNING 상태여야 함

# 2. Baudrate 확인 (송신기 설정과 일치해야 함)
ip -details link show can0

# 3. 에러 확인
ip -s link show can0

# 4. 재시작
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

### DBC 파일 검증
```bash
# cantools 설치
pip install cantools

# DBC 파일 파싱 테스트
python3 << EOF
import cantools
db = cantools.database.load_file('iron_md_joystick.dbc')
print("Messages:")
for msg in db.messages:
    print(f"  {msg.name} (0x{msg.frame_id:X})")
EOF
```

## 📝 참고 사항

1. **비상정지**: 두 개의 Emergency Stop 신호가 있음 (bit 6, 7) - 중복 안전 기능
2. **스위치 번호**: S10-S16은 정의되지 않음 (문서에 없음)
3. **LCD 기능**: 수신 전용이며 옵션 기능 (현재 미사용)
4. **Run Counter**: 0-15 범위로 순환하며 메시지 수신 확인용
5. **TRX Code**: 송수신기 구분 (1=송신기, 0=수신기)

## 📚 관련 파일
- DBC 파일: `/home/koceti/ros2_ws/iron_md_joystick.dbc`
- 문서: `/home/koceti/ros2_ws/IRON-MD_CAN_Specification.md`
- 모니터 스크립트: `/home/koceti/ros2_ws/can_monitor.sh`
