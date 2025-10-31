# ROS 2 프로젝트 리팩토링 계획

## 1. 목표

기존 ROS 2 워크스페이스의 복잡성을 줄이고, ROS 2 표준 설계에 맞춰 모듈성과 유지보수성을 향상시키는 것을 목표로 한다. 각 하드웨어의 제어를 독립적인 드라이버 노드로 분리하고, `ros2_control` 프레임워크를 도입하여 제어 로직을 표준화한다.

## 2. 제안 구조

```
src/
├── rebar_robot_description/       # 로봇 모델 (URDF)
├── rebar_robot_bringup/           # 시스템 실행 (Launch)
├── rebar_robot_control/           # 로봇 제어기 (ros2_control)
├── rebar_robot_teleop/            # 조종기 및 상위 로직
└── hardware/                      # 하드웨어 드라이버
    ├── rmd_driver/
    ├── pololu_driver/
    ├── seengrip_driver/
    ├── iron_md_driver/
    └── ezi_io_driver/
```

---

## 3. 단계별 실행 계획 및 체크리스트

### Phase 1: 하드웨어 드라이버 패키지 생성 (입력/센서)

각 하드웨어와 직접 통신하는 노드를 생성하여 ROS 토픽으로 데이터를 발행하도록 분리합니다.

- [ ] `hardware` 디렉토리 생성
- [ ] **`iron_md_driver` 패키지 생성**
    - [ ] `iron_md_driver_node` 작성
        - 기능: `can3`에서 CAN 메시지 수신
        - 발행 토픽: `sensor_msgs/Joy` (조이스틱), `rebar_robot_msgs/SwitchState` (커스텀 메시지, 스위치 상태)
- [ ] **`ezi_io_driver` 패키지 생성**
    - [ ] `ezi_io_driver_node` 작성
        - 기능: Modbus TCP로 EZI-IO 상태 읽기
        - 발행 토픽: `/limit_switches/z_max` 등 `std_msgs/Bool` 타입의 토픽들

### Phase 2: `ros2_control` 도입 준비

`ros2_control`을 사용하기 위한 기본 환경을 설정합니다.

- [ ] **`rebar_robot_description` 패키지 생성**
    - [ ] `rebar_robot.urdf.xacro` 파일 작성
        - `ros2_control` 태그를 포함하여 각 관절(joint)과 하드웨어 인터페이스(hardware interface) 정의
- [ ] **`rmd_driver` 패키지 생성 (`ros2_control` 연동)**
    - [ ] `rmd_hardware_interface.py` 작성
        - `ros2_control`의 `HardwareInterface` 클래스 상속
        - `read()`: `can2`에서 모터 상태(위치/속도)를 읽어 `ros2_control`에 전달
        - `write()`: `ros2_control`에서 받은 목표 값(위치/속도)을 CAN 명령으로 변환하여 `can2`로 전송
- [ ] **`rebar_robot_control` 패키지 생성**
    - [ ] `config/rebar_robot_controllers.yaml` 파일 작성
        - `diff_drive_controller` 설정 (주행용)
        - `joint_trajectory_controller` 설정 (5개 관절용)

### Phase 3: 하드웨어 드라이버 패키지 생성 (출력/액추에이터)

`ros2_control`로 관리되지 않는 나머지 액추에이터의 드라이버를 생성합니다.

- [ ] **`pololu_driver` 패키지 생성**
    - [ ] `pololu_driver_node` 작성
        - 구독 토픽: `/trigger/command` (`std_msgs/String` 등)
        - 기능: 토픽을 받으면 `SmcCmd`를 `subprocess`로 실행
- [ ] **`seengrip_driver` 패키지 생성**
    - [ ] `seengrip_driver_node` 작성
        - 구독 토픽: `/gripper/command` (`control_msgs/action/GripperCommand` 액션 서버 추천)
        - 기능: 토픽/액션 요청을 받아 Modbus RTU 명령 전송

### Phase 4: 상위 로직 및 통합

분리된 드라이버와 제어기를 사용하여 상위 응용 프로그램을 재작성하고 전체 시스템을 통합합니다.

- [ ] **`rebar_robot_teleop` 패키지 생성**
    - [ ] `rebar_teleop_node` 작성 (기존 `iron_md_teleop_node` 대체)
        - 구독 토픽: `/joy`, `/limit_switches/*` 등 (드라이버 노드들이 발행하는 토픽)
        - 발행 토픽: `/diff_drive_controller/cmd_vel_unstamped`, `/joint_trajectory_controller/joint_trajectory`, `/trigger/command` 등
        - **제거**: CAN, Modbus, `subprocess` 등 하드웨어 직접 제어 코드 모두 제거
- [ ] **`rebar_robot_bringup` 패키지 생성**
    - [ ] `rebar_robot.launch.py` 작성
        - 모든 드라이버 노드 실행
        - `ros2_control` 노드 및 컨트롤러 로드
        - `rebar_teleop_node` 실행

---

## 4. 완료 기준

- [ ] 모든 기존 기능(주행, 관절 이동, 작업 시퀀스, 브레이크, 비상정지)이 새 구조에서 정상적으로 동작한다.
- [ ] `src` 디렉토리의 최상단에 `rebar_control`, `rmd_robot_control` 등 기존 패키지가 존재하지 않는다.
- [ ] 모든 파라미터는 `.yaml` 파일로 관리되며, launch 파일에서 로드된다.
- [ ] `integrated_control.sh`와 같은 쉘 스크립트 대신 `rebar_robot.launch.py`로 전체 시스템이 실행된다.
