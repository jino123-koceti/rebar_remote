#!/usr/bin/env python3
"""
철근 결속 로봇 텔레옵 노드 (Iron-MD CAN 조종기)
Iron-MD 무선 조종기 CAN 메시지를 수신하여 로봇 제어

CAN 통신:
- 0x1E4 (484): 조이스틱 아날로그 데이터 (50ms)
- 0x2E4 (740): 스위치 및 상태 (50ms)
- 0x764 (1892): Heartbeat (300ms)

조종기 매핑:
[아날로그 조이스틱 - 연속 제어]
- Joystick_3 (AN3): 하부체 좌우회전 (0x141, 0x142) - 속도는 전후진의 1/2
- Joystick_4 (AN4): 하부체 전후진 (0x141, 0x142)
- Joystick_1 (AN1): 상부체 X축 속도 제어 (0x144)
- Joystick_2 (AN2): 상부체 Y축 속도 제어 (0x145)

[3단 스위치 - 토글형]
- S19-S20: 모드 선택 (S19=Remote, S20=Automatic)
- S17-S18: 횡이동 (S17=+360도, S18=-360도) 0x143
- S21-S22: 작업 시퀀스 (S21=하강→닫힘, S22=트리거→열림→상승)
- S23-S24: Yaw 회전 (S23=+30도, S24=-30도) 0x147

[일반 스위치]
- S13: 브레이크 해제/잠금
- S14: 위치 리셋
- Emergency_Stop: 비상 정지
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray, Int32, Bool
from std_srvs.srv import Trigger
import can
import struct
import threading
import math


class IronMDTeleopNode(Node):
    """Iron-MD CAN 조종기로 로봇 단동 제어"""
    
    def __init__(self):
        super().__init__('iron_md_teleop')
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can3')  # Iron-MD 조종기용 can3
        self.declare_parameter('can_baudrate', 250000)
        # 최대 속도 제한 (양쪽 모터 떨림 방지를 위해 낮춤)
        self.declare_parameter('max_linear_speed', 0.3)  # 0.5 → 0.3 m/s
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('xyz_step_size', 10.0)  # degree, 10도 per command (조이스틱 연속 제어)
        self.declare_parameter('lateral_move_distance', 50.0)  # degree, 50도 per step (횡이동)
        self.declare_parameter('z_work_distance', 100.0)  # degree, 100도 for work sequence
        self.declare_parameter('yaw_rotation_angle', 30.0)  # degrees, 30도
        self.declare_parameter('trigger_duration', 0.5)  # seconds
        self.declare_parameter('gripper_open_position', 0)  # 그리퍼 열림
        self.declare_parameter('gripper_close_position', 2000)  # 그리퍼 닫힘
        self.declare_parameter('joystick_center', 127)  # 중립값
        self.declare_parameter('joystick_deadzone', 20)  # 데드존
        self.declare_parameter('debug_mode', False)  # 디버그 모드 (터미널 출력 상세화)
        
        self.can_interface = self.get_parameter('can_interface').value
        self.can_baudrate = self.get_parameter('can_baudrate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.xyz_step = self.get_parameter('xyz_step_size').value
        self.lateral_distance = self.get_parameter('lateral_move_distance').value
        self.z_work_distance = self.get_parameter('z_work_distance').value
        self.yaw_angle = self.get_parameter('yaw_rotation_angle').value
        self.trigger_duration = self.get_parameter('trigger_duration').value
        self.gripper_open = self.get_parameter('gripper_open_position').value
        self.gripper_close = self.get_parameter('gripper_close_position').value
        self.joy_center = self.get_parameter('joystick_center').value
        self.joy_deadzone = self.get_parameter('joystick_deadzone').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # ROS2 발행자들
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint1_pub = self.create_publisher(Float64MultiArray, '/joint_1/position', 10)  # 0x143 Lateral (횡이동)
        self.joint2_pub = self.create_publisher(Float32, '/joint_2/speed', 10)  # 0x144 X-axis 속도
        self.joint3_pub = self.create_publisher(Float32, '/joint_3/speed', 10)  # 0x145 Y-axis 속도
        self.joint4_pub = self.create_publisher(Float64MultiArray, '/joint_4/position', 10)  # 0x146 Z-axis (상하)
        self.joint5_pub = self.create_publisher(Float64MultiArray, '/joint_5/position', 10)  # 0x147 Yaw (회전)
        self.trigger_pub = self.create_publisher(Float32, '/motor_0/vel', 10)
        self.gripper_pos_pub = self.create_publisher(Float32, '/gripper/position', 10)
        self.gripper_cmd_pub = self.create_publisher(Int32, '/gripper/command', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # 조이스틱 및 스위치 상태
        self.joystick_data = {
            'AN1': 127,  # X축 (상부체)
            'AN2': 127,  # Y축 (상부체)
            'AN3': 127,  # 주행 전후진 (하부체)
            'AN4': 127,  # (예비)
        }
        
        self.switch_data = {
            'S00': 0, 'S01': 0, 'S02': 0, 'S03': 0,
            'S06': 0, 'S07': 0, 'S08': 0, 'S09': 0,
            'S13': 0,  # 브레이크 해제 버튼
            'S14': 0,  # 드라이브 모터 홈잉
            'S17': 0, 'S18': 0, 'S19': 0, 'S20': 0,
            'S21': 0, 'S22': 0, 'S23': 0, 'S24': 0,
            'Emergency_Stop_Active': 0,
            'Emergency_Stop_Release': 1,
            'TX_Connected': 0,
        }
        
        # 이전 스위치 상태 (엣지 감지)
        self.prev_switches = self.switch_data.copy()
        
        # 현재 위치 (degree 단위)
        self.current_positions = {
            'lateral': 0.0,  # 횡이동 (0x143) - degree
            'x': 0.0,  # X축 (0x144) - degree
            'y': 0.0,  # Y축 (0x145) - degree
            'z': 0.0,  # Z축 (0x146) - degree
            'yaw': 0.0,  # Yaw (0x147) - degree
        }
        
        # 제어 모드
        self.control_mode = 'remote'  # 'remote' or 'automatic'
        
        # 작업 시퀀스 상태
        self.work_sequence_active = False
        
        # 트리거 타이머
        self.trigger_timer = None
        
        # 안전 플래그
        self.emergency_stopped = False

        # 브레이크 상태 (True=해제됨, False=잠김)
        self.brake_released = False
        
        # 마지막 발행 값 (중복 방지)
        self.last_cmd_sent = {'linear': 0.0, 'angular': 0.0}
        
        # 제어 루프 타이머 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # 디버그 출력용 타이머 (1Hz)
        if self.debug_mode:
            self.debug_timer = self.create_timer(1.0, self.print_status)

        # 브레이크 제어 서비스 클라이언트 생성
        self.brake_release_client = self.create_client(Trigger, 'safe_brake_release')
        self.brake_lock_client = self.create_client(Trigger, 'safe_brake_lock')

        # 모터 위치 구독 (0x143 = joint_1, 0x146 = joint_4, 0x147 = joint_5)
        self.motor_position_sub = self.create_subscription(
            Float32,
            '/motor_0x143_position',
            lambda msg: self.motor_position_callback(msg, 'lateral'),
            10
        )
        
        self.motor_z_position_sub = self.create_subscription(
            Float32,
            '/motor_0x146_position',
            lambda msg: self.motor_position_callback(msg, 'z'),
            10
        )
        
        self.motor_yaw_position_sub = self.create_subscription(
            Float32,
            '/motor_0x147_position',
            lambda msg: self.motor_position_callback(msg, 'yaw'),
            10
        )

        # EZI-IO 리미트 센서 구독
        self.limit_sensor_in00 = False  # IN00 상태 (Y축 원점 리미트, 0x145)
        self.limit_sensor_in01 = False  # IN01 상태 (Y축 최대 리미트, 0x145)
        self.limit_sensor_in02 = False  # IN02 상태 (X축 홈 리미트, 0x144)
        self.limit_sensor_in03 = False  # IN03 상태 (X축 최대 리미트, 0x144)
        self.limit_sensor_in05 = False  # IN05 상태 (Z축 상단 리미트)
        self.limit_sensor_in06 = False  # IN06 상태 (Z축 하단 리미트)

        self.limit_sensor_in00_sub = self.create_subscription(
            Bool,
            '/limit_sensors/y_max',  # EZI-IO IN00 (Y축 원점)
            self.limit_sensor_in00_callback,
            10
        )

        self.limit_sensor_in01_sub = self.create_subscription(
            Bool,
            '/limit_sensors/y_min',  # EZI-IO IN01 (Y축 최대)
            self.limit_sensor_in01_callback,
            10
        )

        self.limit_sensor_in02_sub = self.create_subscription(
            Bool,
            '/limit_sensors/x_min',  # EZI-IO IN02 (X축 홈)
            self.limit_sensor_in02_callback,
            10
        )

        self.limit_sensor_in03_sub = self.create_subscription(
            Bool,
            '/limit_sensors/x_max',  # EZI-IO IN03 (X축 최대)
            self.limit_sensor_in03_callback,
            10
        )

        self.limit_sensor_in05_sub = self.create_subscription(
            Bool,
            '/limit_sensors/z_min',  # EZI-IO IN05 (Z축 상단)
            self.limit_sensor_in05_callback,
            10
        )

        self.limit_sensor_in06_sub = self.create_subscription(
            Bool,
            '/limit_sensors/z_max',  # EZI-IO IN06 (Z축 하단)
            self.limit_sensor_in06_callback,
            10
        )

        # 초기 위치 읽기 완료 플래그
        self.initial_position_read = {'lateral': False, 'z': False, 'yaw': False}
        
        # Z축 동작 상태
        self.z_moving_to_limit = False  # 상단 리미트까지 이동 중
        self.z_moving_down = False  # 하강 중
        self.z_target_position = 0.0  # 목표 위치
        
        # 작업 시퀀스 상태
        self.s21_sequence_active = False  # S21 시퀀스 진행 중
        self.s21_sequence_timer = None  # S21 시퀀스 타이머
        self.s22_sequence_active = False  # S22 시퀀스 진행 중
        self.s22_sequence_timer = None  # S22 시퀀스 타이머
        self.s22_sequence_step = 0  # S22 시퀀스 단계
        
        # 드라이브 모터(0x141/0x142) 홈잉 상태
        self.homing_active = False  # 홈잉 진행 중
        self.homing_state = 'idle'  # idle, releasing_sensor, searching_sensor, final_approach
        self.homing_initial_in02 = False  # 초기 IN02 상태 저장

        # 홈잉 속도 설정
        self.homing_speed_slow = 0.02  # 저속: 0.02 m/s (정밀 제어)
        self.homing_speed_medium = 0.05  # 중속: 0.05 m/s (빠른 이동)

        # CAN 버스 초기화 (모든 데이터 구조 초기화 후)
        try:
            # CAN3: Iron-MD 조종기 수신용
            self.can_bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.can_baudrate
            )
            self.get_logger().info(f'CAN bus opened: {self.can_interface} @ {self.can_baudrate} bps')
        except Exception as e:
            self.get_logger().error(f'Failed to open CAN bus: {e}')
            raise

        # CAN 수신 스레드 시작 (마지막에)
        self.can_thread = threading.Thread(target=self.can_receiver_thread, daemon=True)
        self.can_thread.start()

        self.get_logger().info(f'Iron-MD Teleop node started (CAN: {self.can_interface})')
        if self.debug_mode:
            self.get_logger().info('DEBUG MODE: Verbose logging enabled')
        self.print_help()
    
    def print_help(self):
        """조종기 매핑 도움말 (간소화)"""
        self.get_logger().info('Iron-MD Remote Controller Mapping:')
        self.get_logger().info('  AN4: Forward/Backward (0x141,0x142), AN3: Left/Right turn (0x141,0x142)')
        self.get_logger().info('  AN1: X-axis (0x144), AN2: Y-axis (0x145)')
        self.get_logger().info('  S17/S18: Lateral move ±360deg (0x143)')
        self.get_logger().info('  S21/S22: Work sequence (Z-axis + gripper)')
        self.get_logger().info('  S23/S24: Yaw rotation ±30deg (0x147)')
        self.get_logger().info('  S13: Brake toggle, S14: Homing, S15: Reset position') 
    
    def can_receiver_thread(self):
        """CAN 메시지 수신 스레드"""
        while rclpy.ok():
            try:
                msg = self.can_bus.recv(timeout=1.0)
                if msg is not None:
                    self.process_can_message(msg)
            except Exception as e:
                self.get_logger().error(f'CAN receive error: {e}')
    
    def process_can_message(self, msg):
        """CAN 메시지 파싱"""
        can_id = msg.arbitration_id
        data = msg.data
        
        if can_id == 0x1E4:  # 484: Joystick Data
            self.parse_joystick_data(data)
        
        elif can_id == 0x2E4:  # 740: Switch Status
            self.parse_switch_status(data)
        
        elif can_id == 0x764:  # 1892: Heartbeat
            pass  # Heartbeat는 연결 상태 확인용
    
    def parse_joystick_data(self, data):
        """조이스틱 데이터 파싱 (0x1E4)"""
        if len(data) >= 4:
            self.joystick_data['AN1'] = data[0]  # Joystick 1
            self.joystick_data['AN2'] = data[1]  # Joystick 2
            self.joystick_data['AN3'] = data[2]  # Joystick 3
            self.joystick_data['AN4'] = data[3]  # Joystick 4
            
            if self.debug_mode:
                self.get_logger().debug(
                    f'📊 조이스틱: AN1={data[0]:3d} AN2={data[1]:3d} '
                    f'AN3={data[2]:3d} AN4={data[3]:3d}'
                )
    
    def parse_switch_status(self, data):
        """스위치 상태 파싱 (0x2E4)"""
        if len(data) < 8:
            return
        
        # Byte 0: Start, Power, Engine, Emergency, S13
        byte0 = data[0]
        self.switch_data['S13'] = (byte0 >> 2) & 0x01  # 브레이크 해제 버튼
        self.switch_data['Emergency_Stop_Release'] = (byte0 >> 6) & 0x01
        self.switch_data['Emergency_Stop_Active'] = (byte0 >> 7) & 0x01
        
        # Byte 1: S00-S07
        byte1 = data[1]
        self.switch_data['S06'] = (byte1 >> 0) & 0x01
        self.switch_data['S07'] = (byte1 >> 1) & 0x01
        self.switch_data['S02'] = (byte1 >> 4) & 0x01
        self.switch_data['S03'] = (byte1 >> 5) & 0x01
        self.switch_data['S01'] = (byte1 >> 6) & 0x01
        self.switch_data['S00'] = (byte1 >> 7) & 0x01
        
        # Byte 2: S08-S09, S14
        byte2 = data[2]
        prev_s14 = self.switch_data.get('S14', 0)

        self.switch_data['S08'] = (byte2 >> 0) & 0x01
        self.switch_data['S09'] = (byte2 >> 1) & 0x01
        self.switch_data['S14'] = (byte2 >> 3) & 0x01  # 홈잉 버튼 (0x62→0x6A)

        # S14 변화 감지 시 디버그 로그 (모든 비트 출력)
        if prev_s14 != self.switch_data['S14']:
            self.get_logger().info(
                f'🔍 [CAN DEBUG] S14 변화: {prev_s14} → {self.switch_data["S14"]}, '
                f'byte2=0x{byte2:02X} (binary: {bin(byte2)[2:].zfill(8)}, '
                f'bit0={byte2&0x01}, bit1={(byte2>>1)&0x01}, bit2={(byte2>>2)&0x01}, '
                f'bit3={(byte2>>3)&0x01}, bit4={(byte2>>4)&0x01}, bit5={(byte2>>5)&0x01}, '
                f'bit6={(byte2>>6)&0x01}, bit7={(byte2>>7)&0x01})'
            )
        
        # Byte 3: S17-S24
        byte3 = data[3]
        prev_s21 = self.switch_data.get('S21', 0)
        prev_s22 = self.switch_data.get('S22', 0)
        
        self.switch_data['S23'] = (byte3 >> 0) & 0x01
        self.switch_data['S24'] = (byte3 >> 1) & 0x01
        self.switch_data['S21'] = (byte3 >> 2) & 0x01
        self.switch_data['S22'] = (byte3 >> 3) & 0x01
        self.switch_data['S19'] = (byte3 >> 4) & 0x01
        self.switch_data['S20'] = (byte3 >> 5) & 0x01
        self.switch_data['S17'] = (byte3 >> 6) & 0x01
        self.switch_data['S18'] = (byte3 >> 7) & 0x01
        
        # S21/S22 변화 감지 시 디버그 로그
        if prev_s21 != self.switch_data['S21']:
            self.get_logger().info(
                f'[CAN DEBUG] S21 변화: {prev_s21} -> {self.switch_data["S21"]}, '
                f'byte3=0x{byte3:02X} (binary: {bin(byte3)[2:].zfill(8)})'
            )
        if prev_s22 != self.switch_data['S22']:
            self.get_logger().info(
                f'[CAN DEBUG] S22 변화: {prev_s22} -> {self.switch_data["S22"]}, '
                f'byte3=0x{byte3:02X} (binary: {bin(byte3)[2:].zfill(8)})'
            )
        
        # Byte 6: TX Connected
        if len(data) >= 7:
            byte6 = data[6]
            self.switch_data['TX_Connected'] = (byte6 >> 6) & 0x01
    
    def switch_pressed(self, switch_name):
        """스위치 엣지 감지 (Rising Edge)"""
        current = self.switch_data.get(switch_name, 0)
        previous = self.prev_switches.get(switch_name, 0)
        return current == 1 and previous == 0
    
    def normalize_joystick(self, value):
        """조이스틱 값 정규화 (0-255 -> -1.0 to 1.0)"""
        centered = value - self.joy_center
        
        # 데드존 적용
        if abs(centered) < self.joy_deadzone:
            return 0.0
        
        # 정규화
        if centered > 0:
            return centered / (255 - self.joy_center)
        else:
            return centered / self.joy_center
    
    def control_loop(self):
        """제어 루프 (20Hz)"""
        # 비상 정지 체크
        if self.switch_data['Emergency_Stop_Active'] == 1:
            if not self.emergency_stopped:
                self.emergency_stop()
            return
        elif self.emergency_stopped and self.switch_data['Emergency_Stop_Release'] == 1:
            self.emergency_stopped = False
            self.get_logger().info('Emergency stop released (hardware)')
        
        if self.emergency_stopped:
            return
        
        # 연결 상태 확인
        if self.switch_data['TX_Connected'] == 0:
            # 송신기 연결 안됨 - 모든 모터 정지
            self.publish_zero_velocity()
            return
        
        # 제어 모드 확인
        self.update_control_mode()
        
        if self.control_mode != 'remote':
            # Automatic 모드일 경우 상위제어 패키지에 제어권 위임
            return
        
        # Remote Control 모드
        # 1. 주행 제어 (연속) - AN3
        self.handle_driving()
        
        # 2. XYZ 스테이지 (연속) - AN1, AN2
        self.handle_xyz_stage()
        
        # 3. 횡이동 (스텝) - S17, S18
        self.handle_lateral_move()
        
        # 4. 작업 시퀀스 - S21, S22
        self.handle_work_sequence()
        
        # 5. Yaw 회전 - S23, S24
        self.handle_yaw_rotation()
        
        # 6. 브레이크 해제 - S13
        self.handle_brake_release()

        # 7. 홈잉 - S14
        self.handle_homing()

        # 이전 상태 저장
        self.prev_switches = self.switch_data.copy()
    
    def update_control_mode(self):
        """제어 모드 업데이트 (S19-S20)"""
        if self.switch_data['S19'] == 1:
            if self.control_mode != 'remote':
                self.control_mode = 'remote'
                self.get_logger().info('Remote Control mode')
        elif self.switch_data['S20'] == 1:
            if self.control_mode != 'automatic':
                self.control_mode = 'automatic'
                self.get_logger().info('🤖 Automatic Control 모드 (상위제어 대기)')
    
    def handle_driving(self):
        """주행 제어 (AN3: 좌우회전, AN4: 전후진)"""
        # 홈잉 중일 때는 조이스틱 제어 무시
        if self.homing_active:
            return

        # AN4: 전후진 (AN4+ = 전진, AN4- = 후진)
        linear = self.normalize_joystick(self.joystick_data['AN4'])

        # AN3: 좌우회전 (AN3- = 좌회전, AN3+ = 우회전)
        angular = self.normalize_joystick(self.joystick_data['AN3'])
        
        # 로그: 조이스틱 RAW 값 (DEBUG 모드만)
        if self.debug_mode and (abs(linear) > 0.01 or abs(angular) > 0.01):
            self.get_logger().debug(
                f'Joystick: AN3={self.joystick_data["AN3"]}, AN4={self.joystick_data["AN4"]}, '
                f'linear={linear:.3f}, angular={angular:.3f}'
            )
        
        # 값 변화가 있을 때만 발행 (중복 방지)
        if (abs(linear - self.last_cmd_sent['linear']) > 0.01 or
            abs(angular - self.last_cmd_sent['angular']) > 0.01):
            twist = Twist()
            # AN4(전후진): 전체 속도 (0.5 -> 1.0으로 2배 증가)
            twist.linear.x = linear * self.max_linear * 1.0
            # AN3(좌우회전): 기존 전후진 속도 적용 (전체 속도)
            twist.angular.z = angular * self.max_linear
            
            # 로그: ROS2 토픽 발행 (DEBUG 모드만)
            if self.debug_mode and (abs(linear) > 0.01 or abs(angular) > 0.01):
                self.get_logger().debug(
                    f'cmd_vel pub: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}'
                )
            
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_sent['linear'] = linear
            self.last_cmd_sent['angular'] = angular
    
    def handle_xyz_stage(self):
        """XYZ 스테이지 제어 (AN1: X축 0x144, AN2: Y축 0x145) - 속도 제어"""
        # 홈잉 중일 때는 XYZ 스테이지 제어 무시
        if self.homing_active:
            return

        import can
        import struct

        # X축: AN1 -> 0x144 속도 제어 (방향 반전)
        x_value = self.normalize_joystick(self.joystick_data['AN1'])

        # 안전: 리미트 센서 체크 및 방향 제한
        if abs(x_value) > 0.1:
            # IN02 (홈) ON이면 양수(+) 방향 차단 (반전 고려)
            if self.limit_sensor_in02 and x_value > 0:
                self.get_logger().warning('IN02 sensor ON: blocking positive direction')
                x_value = 0.0

            # IN03 (최대) ON이면 음수(-) 방향 차단 (반전 고려)
            if self.limit_sensor_in03 and x_value < 0:
                self.get_logger().warning('IN03 sensor ON: blocking negative direction')
                x_value = 0.0

        if abs(x_value) > 0.1:
            try:
                x_speed_dps = x_value * 200.0  # 최대 200 dps
                speed_control = int(x_speed_dps * 100)  # 0.01 dps/LSB

                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x144,
                    data=[
                        0xA2,  # Speed Control Command
                        0x64,  # 100% max torque
                        0x00,
                        0x00,
                        speed_control & 0xFF,
                        (speed_control >> 8) & 0xFF,
                        (speed_control >> 16) & 0xFF,
                        (speed_control >> 24) & 0xFF
                    ],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()

                if self.debug_mode:
                    self.get_logger().debug(f'X-axis (0x144) speed: {x_speed_dps:.1f} dps')
            except Exception as e:
                self.get_logger().error(f'X축 속도 제어 실패: {e}')
        else:
            # 조이스틱이 중립이면 정지
            try:
                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x144,
                    data=[0xA2, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()
            except:
                pass
        
        # Y축: AN2 -> 0x145 속도 제어 (반전 없음)
        y_value = -self.normalize_joystick(self.joystick_data['AN2'])

        # 안전: 리미트 센서 체크 및 방향 제한
        if abs(y_value) > 0.1:
            # IN00 (원점) ON이면 양수(+) 방향 차단 (반대로 수정)
            if self.limit_sensor_in00 and y_value > 0:
                self.get_logger().warning('IN00 (Y-axis home) sensor ON: blocking positive direction')
                y_value = 0.0

            # IN01 (최대) ON이면 음수(-) 방향 차단 (반대로 수정)
            if self.limit_sensor_in01 and y_value < 0:
                self.get_logger().warning('IN01 (Y-axis max) sensor ON: blocking negative direction')
                y_value = 0.0

        if abs(y_value) > 0.1:
            try:
                y_speed_dps = y_value * 200.0  # 최대 200 dps
                speed_control = int(y_speed_dps * 100)  # 0.01 dps/LSB

                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x145,
                    data=[
                        0xA2,  # Speed Control Command
                        0x64,  # 100% max torque
                        0x00,
                        0x00,
                        speed_control & 0xFF,
                        (speed_control >> 8) & 0xFF,
                        (speed_control >> 16) & 0xFF,
                        (speed_control >> 24) & 0xFF
                    ],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()

                if self.debug_mode:
                    self.get_logger().debug(f'Y-axis (0x145) speed: {y_speed_dps:.1f} dps')
            except Exception as e:
                self.get_logger().error(f'Y축 속도 제어 실패: {e}')
        else:
            # 조이스틱이 중립이면 정지
            try:
                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x145,
                    data=[0xA2, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()
            except:
                pass
    
    def handle_lateral_move(self):
        """횡이동 제어 (S17/S18: ±360도 회전) 0x143"""
        # 홈잉 중일 때는 횡이동 제어 무시
        if self.homing_active:
            return
        
        if self.switch_pressed('S17'):
            # 양의 방향 360도 회전
            self.current_positions['lateral'] += 360.0  # degree
            self.publish_joint_position('lateral', self.joint1_pub)  # joint_1 = 0x143
            self.get_logger().info(f'>> 횡이동 +360도 (0x143, 누적: {self.current_positions["lateral"]:.1f}도)')

        elif self.switch_pressed('S18'):
            # 음의 방향 360도 회전
            self.current_positions['lateral'] -= 360.0  # degree
            self.publish_joint_position('lateral', self.joint1_pub)  # joint_1 = 0x143
            self.get_logger().info(f'<< 횡이동 -360도 (0x143, 누적: {self.current_positions["lateral"]:.1f}도)')

    def handle_work_sequence(self):
        """Z축 제어 (S21: 홈→하강→닫힘, S22: 트리거→열림→상승) 0x146"""
        # 홈잉 중일 때는 Z축 제어 무시
        if self.homing_active:
            return

        if self.switch_pressed('S21'):
            self.get_logger().info('[DEBUG] S21 button pressed detected!')
            # S21 시퀀스 시작: 그리퍼 Home -> Z축 하강 -> 그리퍼 닫기
            if not self.s21_sequence_active:
                self.s21_sequence_active = True
                self.s21_sequence_step = 0

                # 1. 그리퍼 Home 명령
                self.get_logger().info('[S21 Sequence] 1단계: Gripper Home command')
                gripper_cmd = Int32()
                gripper_cmd.data = 3  # DRV_CMD_HOME
                self.gripper_cmd_pub.publish(gripper_cmd)
                self.get_logger().info(f'  -> Published /gripper/command: {gripper_cmd.data} (HOME)')

                # 2초 후 Z축 하강 (그리퍼 Home 완료 대기)
                self.s21_sequence_timer = self.create_timer(2.0, self.s21_sequence_move_down)
            else:
                self.get_logger().warning('WARNING: S21 sequence already in progress')

        elif self.switch_pressed('S22'):
            self.get_logger().info('[DEBUG] S22 button pressed detected!')
            # S22 시퀀스 시작: 트리거 -> 그리퍼 열기 -> Z축 상승
            if not self.s22_sequence_active:
                self.s22_sequence_active = True
                self.s22_sequence_step = 0

                # 1. 트리거 동작
                self.get_logger().info('[S22 Sequence] 1단계: 트리거 동작 시작')
                self.trigger_pull()

                # 3초 후 그리퍼 열기 (트리거 완료 대기: 1초 당김 + 1초 되돌림 + 1초 여유)
                self.s22_sequence_timer = self.create_timer(3.0, self.s22_sequence_open_gripper)
            else:
                self.get_logger().warning('WARNING: S22 sequence already in progress')
    
    def s21_sequence_move_down(self):
        """S21 시퀀스 2단계: Z축 하강"""
        if self.s21_sequence_timer:
            self.s21_sequence_timer.cancel()
            self.s21_sequence_timer = None

        # 2단계: Z축 음의 방향 약 3.06회전 (1100도) 하강
        self.z_moving_down = True
        self.z_moving_to_limit = False
        self.current_positions['z'] -= 1100.0  # degree
        self.publish_joint_position('z', self.joint4_pub)  # joint_4 = 0x146
        self.get_logger().info(f'[S21 Sequence] 2단계: Z축 -약 3.06회전(1100°) 하강 (0x146, 누적: {self.current_positions["z"]:.1f}도)')

        # 6.0초 후 그리퍼 닫기 (하강 완료 대기: 1100도 하강 시간 고려, 100dps 속도 기준 약 11초)
        self.s21_sequence_timer = self.create_timer(6.0, self.s21_sequence_gripper_close)

    def s21_sequence_gripper_close(self):
        """S21 시퀀스 3단계: 그리퍼 완전히 닫기"""
        if self.s21_sequence_timer:
            self.s21_sequence_timer.cancel()
            self.s21_sequence_timer = None

        self.get_logger().info('[S21 Sequence] 3단계: Gripper close command')

        # 그리퍼 완전히 닫기 (1.0 = 완전 닫힘)
        gripper_msg = Float32()
        gripper_msg.data = 1.0
        self.gripper_pos_pub.publish(gripper_msg)
        self.get_logger().info(f'  -> Published /gripper/position: {gripper_msg.data}')

        # 시퀀스 완료
        self.s21_sequence_active = False
        self.z_moving_down = False
        self.get_logger().info('[S21 Sequence] Complete - Gripper stays closed')

    def s22_sequence_open_gripper(self):
        """S22 시퀀스 2단계: 그리퍼 완전히 열기"""
        if self.s22_sequence_timer:
            self.s22_sequence_timer.cancel()
            self.s22_sequence_timer = None

        self.get_logger().info('[S22 Sequence] 2단계: Gripper open command')

        # 그리퍼 완전히 열기 (0.0 = 완전 열림)
        gripper_msg = Float32()
        gripper_msg.data = 0.0
        self.gripper_pos_pub.publish(gripper_msg)
        self.get_logger().info(f'  -> Published /gripper/position: {gripper_msg.data}')

        # 2초 후 Z축 상승 (그리퍼 열림 완료 대기)
        self.s22_sequence_timer = self.create_timer(2.0, self.s22_sequence_move_up)

    def s22_sequence_move_up(self):
        """S22 시퀀스 3단계: Z축 상승"""
        if self.s22_sequence_timer:
            self.s22_sequence_timer.cancel()
            self.s22_sequence_timer = None

        # 3단계: Z축 원점 리미트까지 상승
        self.z_moving_down = False
        self.z_moving_to_limit = True
        self.current_positions['z'] += 3600.0  # degree (충분히 큰 값)
        self.publish_joint_position('z', self.joint4_pub)  # joint_4 = 0x146
        self.get_logger().info(f'[S22 Sequence] 3단계: Z-axis to home 리미트까지 상승 (0x146, IN05 감지 대기)')

        # 시퀀스 완료
        self.s22_sequence_active = False
        self.get_logger().info('[S22 Sequence] Complete')

    
    def handle_yaw_rotation(self):
        """Yaw 회전 제어 (S23: +30°, S24: -30°) 0x147"""
        # 홈잉 중일 때는 Yaw 제어 무시
        if self.homing_active:
            return
        
        if self.switch_pressed('S23'):
            # 양의 방향 30도 회전 (0x143과 동일한 방식)
            self.current_positions['yaw'] += self.yaw_angle  # degree 단위 직접 처리
            self.publish_joint_position('yaw', self.joint5_pub)  # joint_5 = 0x147
            self.get_logger().info(f'↻  Yaw +30° (0x147, 누적: {self.current_positions["yaw"]:.1f}도)')
        
        elif self.switch_pressed('S24'):
            # 음의 방향 30도 회전 (0x143과 동일한 방식)
            self.current_positions['yaw'] -= self.yaw_angle  # degree 단위 직접 처리
            self.publish_joint_position('yaw', self.joint5_pub)  # joint_5 = 0x147
            self.get_logger().info(f'↺  Yaw -30° (0x147, 누적: {self.current_positions["yaw"]:.1f}도)')
    
    def handle_brake_release(self):
        """브레이크 해제/잠금 토글 (S13) - 위치제어 모터 0x143-0x147"""
        if self.switch_pressed('S13'):
            # 현재 상태 반전
            self.brake_released = not self.brake_released

            if self.brake_released:
                # 브레이크 해제 서비스 호출
                self.get_logger().info('🔓 브레이크 해제 서비스 호출...')
                self.call_brake_service(self.brake_release_client, '해제')
            else:
                # 브레이크 잠금 서비스 호출
                self.get_logger().info('🔒 브레이크 잠금 서비스 호출...')
                self.call_brake_service(self.brake_lock_client, '잠금')

            # LCD 디스플레이에 브레이크 상태 표시
            self.send_lcd_brake_status()

    def call_brake_service(self, client, action_name):
        """브레이크 서비스 호출 (비동기)"""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'WARNING: Brake {action_name} 서비스를 사용할 수 없습니다')
            return

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.brake_service_callback(f, action_name))

    def brake_service_callback(self, future, action_name):
        """브레이크 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 브레이크 {action_name} 완료!')

                # 브레이크 해제 시 초기 위치 읽기 플래그 리셋
                if action_name == '해제':
                    self.initial_position_read = {'lateral': False, 'z': False, 'yaw': False}
                    self.get_logger().info('📍 모터 초기 위치 읽는 중... (1초 대기)')
            else:
                self.get_logger().warning(f'WARNING: Brake {action_name} 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ 브레이크 {action_name} 서비스 오류: {e}')

    def motor_position_callback(self, msg: Float32, motor_type: str):
        """모터 위치 콜백 (0x143: lateral, 0x146: z, 0x147: yaw)"""
        # 초기 위치만 읽고, 이후에는 피드백으로 위치를 덮어쓰지 않음 (누적 오류 방지)
        if motor_type == 'lateral':
            # 초기 위치 읽기 (브레이크 해제 직후 한 번만)
            if not self.initial_position_read[motor_type] and self.brake_released:
                self.current_positions['lateral'] = msg.data
                self.get_logger().info(f'✅ 0x143 (lateral) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
        elif motor_type == 'z':
            # 초기 위치 읽기 (브레이크 해제 직후 한 번만)
            if not self.initial_position_read[motor_type] and self.brake_released:
                self.current_positions['z'] = msg.data
                self.get_logger().info(f'✅ 0x146 (z) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
        elif motor_type == 'yaw':
            # 초기 위치 읽기 (브레이크 해제 직후 한 번만)
            if not self.initial_position_read[motor_type] and self.brake_released:
                self.current_positions['yaw'] = msg.data
                self.get_logger().info(f'✅ 0x147 (yaw) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
    
    def limit_sensor_in05_callback(self, msg: Bool):
        """EZI-IO IN05 리미트 센서 콜백 (Z축 상단 리미트)"""
        prev_state = self.limit_sensor_in05
        self.limit_sensor_in05 = msg.data
        
        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in05:
            status = "ON (감지됨)" if self.limit_sensor_in05 else "OFF"
            self.get_logger().info(f'🔴 IN05 리미트 센서 (상단): {status}')
        
        # 상단 리미트까지 이동 중이고 센서가 ON되면 정지
        if self.z_moving_to_limit and self.limit_sensor_in05:
            self.get_logger().info('✅ 상단 리미트 감지! Z축 긴급 정지')
            self.z_moving_to_limit = False
            # 0x146 모터에 긴급 정지 명령 전송 (CAN2를 통해)
            self.send_motor_emergency_stop(0x146)
            # Z축 위치를 원점(0도)으로 리셋
            self.current_positions['z'] = 0.0
            self.get_logger().info('🏠 Z축 위치 원점(0°)으로 리셋')
    
    def limit_sensor_in06_callback(self, msg: Bool):
        """EZI-IO IN06 리미트 센서 콜백 (Z축 하단 리미트)"""
        prev_state = self.limit_sensor_in06
        self.limit_sensor_in06 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in06:
            status = "ON (감지됨)" if self.limit_sensor_in06 else "OFF"
            self.get_logger().info(f'🔴 IN06 리미트 센서 (하단): {status}')

        # 하강 중이고 센서가 ON되면 정지
        if self.z_moving_down and self.limit_sensor_in06:
            self.get_logger().info('✅ 하단 리미트 감지! Z축 긴급 정지')
            self.z_moving_down = False
            # 0x146 모터에 긴급 정지 명령 전송 (CAN2를 통해)
            self.send_motor_emergency_stop(0x146)

    def limit_sensor_in00_callback(self, msg: Bool):
        """EZI-IO IN00 리미트 센서 콜백 (Y축 원점 리미트, 0x145 음수 제한)"""
        prev_state = self.limit_sensor_in00
        self.limit_sensor_in00 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in00:
            status = "ON (감지됨)" if self.limit_sensor_in00 else "OFF"
            self.get_logger().info(f'🟢 IN00 리미트 센서 (Y축 원점): {status}')

        # 센서 ON되면 0x145 긴급 정지
        if self.limit_sensor_in00 and not prev_state:
            self.get_logger().warning('WARNING: IN00 detected! 0x145 긴급 정지')
            self.send_motor_emergency_stop(0x145)

    def limit_sensor_in01_callback(self, msg: Bool):
        """EZI-IO IN01 리미트 센서 콜백 (Y축 최대 리미트, 0x145 양수 제한)"""
        prev_state = self.limit_sensor_in01
        self.limit_sensor_in01 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in01:
            status = "ON (감지됨)" if self.limit_sensor_in01 else "OFF"
            self.get_logger().info(f'🟡 IN01 리미트 센서 (Y축 최대): {status}')

        # 센서 ON되면 0x145 긴급 정지
        if self.limit_sensor_in01 and not prev_state:
            self.get_logger().warning('WARNING: IN01 detected! 0x145 긴급 정지')
            self.send_motor_emergency_stop(0x145)

    def limit_sensor_in02_callback(self, msg: Bool):
        """EZI-IO IN02 리미트 센서 콜백 (X축 홈 리미트, 0x144 후진 제한)"""
        prev_state = self.limit_sensor_in02
        self.limit_sensor_in02 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in02:
            status = "ON (감지됨)" if self.limit_sensor_in02 else "OFF"
            self.get_logger().info(f'🟢 IN02 리미트 센서 (X축 홈): {status}')

        # 센서 ON되면 0x144 긴급 정지 (홈잉 중이 아닐 때)
        if self.limit_sensor_in02 and not prev_state and not self.homing_active:
            self.get_logger().warning('WARNING: IN02 detected! 0x144 긴급 정지')
            self.send_motor_emergency_stop(0x144)

        # 홈잉 중일 때 센서 상태에 따라 처리
        if not self.homing_active:
            return

        # 상태별 처리
        if self.homing_state == 'releasing_sensor':
            # 센서 해제 중 → IN02 OFF되면 다음 단계로
            if not self.limit_sensor_in02:
                self.get_logger().info('✅ IN02 센서 해제 완료!')
                # 긴급 정지
                self.send_motor_emergency_stop(0x141)
                self.send_motor_emergency_stop(0x142)
                # 다음 단계로: 최종 접근 (수동 취소 방식)
                if hasattr(self, 'homing_timer') and self.homing_timer:
                    self.homing_timer.cancel()
                self.homing_timer = self.create_timer(0.5, self._homing_final_approach_wrapper)

        elif self.homing_state == 'searching_sensor':
            # 센서 탐색 중 → IN02 ON되면 긴급정지
            if self.limit_sensor_in02:
                self.get_logger().info('✅ IN02 센서 감지!')
                # 긴급 정지
                self.send_motor_emergency_stop(0x141)
                self.send_motor_emergency_stop(0x142)
                # 다음 단계로: 센서 해제 (수동 취소 방식)
                if hasattr(self, 'homing_timer') and self.homing_timer:
                    self.homing_timer.cancel()
                self.homing_timer = self.create_timer(0.5, self._homing_release_sensor_wrapper)

        elif self.homing_state == 'final_approach':
            # 최종 접근 중 → IN02 ON되면 홈잉 완료
            if self.limit_sensor_in02:
                self.get_logger().info('✅ 홈 리미트 최종 감지! 드라이브 모터 긴급 정지')
                self.homing_active = False
                self.homing_state = 'idle'
                # 긴급 정지
                self.send_motor_emergency_stop(0x141)
                self.send_motor_emergency_stop(0x142)
                self.get_logger().info('🏠 홈잉 완료!')

    def limit_sensor_in03_callback(self, msg: Bool):
        """EZI-IO IN03 리미트 센서 콜백 (X축 최대 리미트, 0x144 전진 제한)"""
        prev_state = self.limit_sensor_in03
        self.limit_sensor_in03 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in03:
            status = "ON (감지됨)" if self.limit_sensor_in03 else "OFF"
            self.get_logger().info(f'🟡 IN03 리미트 센서 (X축 최대): {status}')

        # 센서 ON되면 0x144 긴급 정지
        if self.limit_sensor_in03 and not prev_state:
            self.get_logger().warning('WARNING: IN03 detected! 0x144 긴급 정지')
            self.send_motor_emergency_stop(0x144)
    
    def send_motor_emergency_stop(self, motor_id):
        """모터 긴급 정지 명령 전송 (0x81)"""
        try:
            import can
            # CAN2 버스로 긴급 정지 명령 전송
            can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
            
            # RMD 긴급 정지 명령: 0x81
            msg = can.Message(
                arbitration_id=motor_id,
                data=[0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            can_bus.send(msg)
            can_bus.shutdown()
            
            self.get_logger().info(f'🛑 모터 0x{motor_id:03X} 긴급 정지 명령 전송')
        except Exception as e:
            self.get_logger().error(f'❌ 긴급 정지 명령 전송 실패: {e}')
    
    def start_homing_sequence(self):
        """드라이브 모터 홈잉 시퀀스 시작"""
        if self.homing_active:
            self.get_logger().warn('WARNING: Homing already in progress')
            return

        self.get_logger().info('🏁 드라이브 모터(0x141/0x142) 홈잉 시작')
        self.homing_active = True

        # 초기 IN02 상태 저장
        self.homing_initial_in02 = self.limit_sensor_in02

        if self.homing_initial_in02:
            # IN02 ON인 경우: 센서 해제 단계부터 시작
            self.get_logger().info('📍 초기 상태: IN02 ON → 센서 해제 단계 시작')
            self.homing_release_sensor()
        else:
            # IN02 OFF인 경우: 센서 탐색 단계부터 시작
            self.get_logger().info('📍 초기 상태: IN02 OFF → 센서 탐색 단계 시작')
            self.homing_search_sensor()

    def homing_release_sensor(self):
        """홈잉: 센서 해제 단계 (저속 전진)"""
        if not self.homing_active:
            return

        self.homing_state = 'releasing_sensor'
        self.get_logger().info(f'➡️  [1단계] 센서 해제: 저속({self.homing_speed_slow} m/s) 전진 중...')

        # 저속 전진
        cmd_vel = Twist()
        cmd_vel.linear.x = self.homing_speed_slow
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def homing_search_sensor(self):
        """홈잉: 센서 탐색 단계 (중속 후진)"""
        if not self.homing_active:
            return

        self.homing_state = 'searching_sensor'
        self.get_logger().info(f'⬅️  [2단계] 센서 탐색: 중속({self.homing_speed_medium} m/s) 후진 중...')

        # 중속 후진
        cmd_vel = Twist()
        cmd_vel.linear.x = -self.homing_speed_medium
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def _homing_release_sensor_wrapper(self):
        """홈잉: 센서 해제 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'homing_timer') and self.homing_timer:
            self.homing_timer.cancel()
            self.homing_timer = None
        self.homing_release_sensor()

    def _homing_final_approach_wrapper(self):
        """홈잉: 최종 접근 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'homing_timer') and self.homing_timer:
            self.homing_timer.cancel()
            self.homing_timer = None
        self.homing_final_approach()

    def homing_final_approach(self):
        """홈잉: 최종 접근 단계 (저속 후진)"""
        if not self.homing_active:
            return

        self.homing_state = 'final_approach'
        self.get_logger().info(f'⬅️  [3단계] 최종 접근: 저속({self.homing_speed_slow} m/s) 후진 중...')

        # 저속 후진
        cmd_vel = Twist()
        cmd_vel.linear.x = -self.homing_speed_slow
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def clear_motor_error(self, motor_id):
        """모터 에러 클리어 (0x9B)"""
        try:
            import can
            
            can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
            
            # RMD 에러 클리어 명령: 0x9B
            msg = can.Message(
                arbitration_id=motor_id,
                data=[0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            can_bus.send(msg)
            can_bus.shutdown()
            
            self.get_logger().info(f'� 모터 0x{motor_id:03X} 에러 클리어')
        except Exception as e:
            self.get_logger().error(f'❌ 에러 클리어 실패: {e}')

    def handle_homing(self):
        """드라이브 모터 홈잉 (S14)"""
        # 디버그: S14 상태 출력
        current = self.switch_data.get('S14', 0)
        previous = self.prev_switches.get('S14', 0)

        if current != previous:
            self.get_logger().info(f'🔍 [DEBUG] S14 상태 변화: {previous} → {current}')

        if self.switch_pressed('S14'):
            self.get_logger().info('🔘 S14 버튼 눌림 감지!')
            self.start_homing_sequence()

    def trigger_pull(self):
        """트리거 당기기 (SmcCmd 사용) - S22 작업 시퀀스에서만 호출"""
        self.get_logger().info('Trigger pull started!')
        
        try:
            import subprocess
            SMC_CMD = '/home/test/ros2_ws/src/smc_linux/SmcCmd'
            DEVICE = '#51FF-7406-4980-4956-3043-1287'  # Pololu 시리얼 넘버
            
            # Resume 명령으로 safe-start 해제
            subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--resume'],
                capture_output=True,
                timeout=2
            )
            
            # Forward at 100% speed (3200) - 트리거 당김
            result = subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--speed', '3200'],
                capture_output=True,
                timeout=2,
                text=True
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'트리거 당김 실패: {result.stderr}')
                return
            
            self.get_logger().info('Trigger pull: forward command sent')

            # 1초 후 역방향 타이머 설정 (수동 취소 방식)
            if hasattr(self, 'trigger_pull_timer') and self.trigger_pull_timer:
                self.trigger_pull_timer.cancel()
            self.trigger_pull_timer = self.create_timer(1.0, self._trigger_reverse_wrapper)

        except Exception as e:
            self.get_logger().error(f'Trigger pull failed: {e}')

    def _trigger_reverse_wrapper(self):
        """트리거 reverse 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'trigger_pull_timer') and self.trigger_pull_timer:
            self.trigger_pull_timer.cancel()
            self.trigger_pull_timer = None
        self.trigger_reverse()
    
    def trigger_reverse(self):
        """트리거 되돌림 (1초 후 호출)"""
        try:
            import subprocess
            SMC_CMD = '/home/test/ros2_ws/src/smc_linux/SmcCmd'
            DEVICE = '#51FF-7406-4980-4956-3043-1287'

            # Resume 명령으로 safe-start 해제 (정지 후 재시작을 위해 필요)
            subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--resume'],
                capture_output=True,
                timeout=2
            )

            # Reverse at 100% speed (-3200) - 트리거 되돌림
            result = subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--speed', '-3200'],
                capture_output=True,
                timeout=2,
                text=True
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'트리거 되돌림 실패: {result.stderr}')
                return
            
            self.get_logger().info('Trigger reverse: backward command sent')

            # 타이머로 자동 정지 설정 (0.5초 후, 수동 취소 방식)
            if self.trigger_timer:
                self.trigger_timer.cancel()
            self.trigger_timer = self.create_timer(self.trigger_duration, self._trigger_release_wrapper)

        except Exception as e:
            self.get_logger().error(f'Trigger reverse failed: {e}')

    def _trigger_release_wrapper(self):
        """트리거 release 래퍼 (타이머 자동 취소)"""
        if self.trigger_timer:
            self.trigger_timer.cancel()
            self.trigger_timer = None
        self.trigger_release()
    
    def trigger_release(self):
        """트리거 해제 (SmcCmd 사용)"""
        try:
            import subprocess
            SMC_CMD = '/home/test/ros2_ws/src/smc_linux/SmcCmd'
            DEVICE = '#51FF-7406-4980-4956-3043-1287'  # Pololu 시리얼 넘버
            
            # Stop motor
            result = subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--speed', '0'],
                capture_output=True,
                timeout=2,
                text=True
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'트리거 해제 실패: {result.stderr}')

            if self.trigger_timer:
                self.trigger_timer.cancel()
                self.trigger_timer = None

            self.get_logger().info('🔫 트리거 해제')
        except Exception as e:
            self.get_logger().error(f'❌ 트리거 해제 실패: {e}')

    def publish_joint_position(self, joint_name, publisher, show_log=True):
        """관절 위치 명령 발행 (degree 단위)"""
        msg = Float64MultiArray()
        # 모든 축이 degree 단위로 통일 (0x143과 동일한 방식)
        msg.data = [self.current_positions[joint_name]]
        publisher.publish(msg)
        
        if show_log:
            self.get_logger().info(
                f'📍 {joint_name.upper()}: {self.current_positions[joint_name]:.2f}°'
            )
    
    def send_lcd_brake_status(self):
        """Iron-MD LCD에 브레이크 상태 표시 (Page 1, Line 0)"""
        try:
            # LCD 표시 메시지 생성 (8자)
            brake_status = "Brake:ON" if not self.brake_released else "Brake:OF"

            # Iron-MD LCD 프로토콜: CAN ID 0x3E4 (996)
            # Byte 0: Page (1 = Page1)
            # Byte 1: Line (0~3)
            # Byte 2-7: ASCII 문자 (6 bytes, 8 char 중 앞 6자)
            lcd_data = bytearray(8)
            lcd_data[0] = 0x01  # Page 1
            lcd_data[1] = 0x00  # Line 0

            # 메시지를 ASCII로 인코딩 (최대 6바이트)
            msg_bytes = brake_status[:6].encode('ascii')
            for i, byte in enumerate(msg_bytes):
                lcd_data[2 + i] = byte

            # CAN 메시지 전송
            msg = can.Message(
                arbitration_id=0x3E4,
                data=bytes(lcd_data),
                is_extended_id=False
            )
            self.can_bus.send(msg)  # CAN3으로 전송 (Iron-MD로 돌려보냄)
            self.get_logger().debug(f'📺 LCD 표시: {brake_status}')

        except Exception as e:
            self.get_logger().error(f'❌ LCD 표시 실패: {e}')

    def publish_zero_velocity(self):
        """모든 모터 정지"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def emergency_stop(self):
        """비상 정지"""
        self.emergency_stopped = True
        self.get_logger().error('🚨 비상 정지 활성화!')
        
        # 모든 모터 정지
        self.publish_zero_velocity()
        self.trigger_release()
        
        # 비상 정지 신호 발행
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
    
    def print_status(self):
        """디버그 모드 상태 출력 (1Hz)"""
        if not self.debug_mode:
            return
        
        # 조이스틱 정규화 값
        an1_norm = self.normalize_joystick(self.joystick_data['AN1'])
        an2_norm = self.normalize_joystick(self.joystick_data['AN2'])
        an3_norm = self.normalize_joystick(self.joystick_data['AN3'])
        an4_norm = self.normalize_joystick(self.joystick_data['AN4'])
        
        # 간소화된 상태 출력 (주요 정보만)
        self.get_logger().info(f'--- Iron-MD Status (DEBUG) ---')
        self.get_logger().info(f'TX: {"Connected" if self.switch_data["TX_Connected"] else "Disconnected"}, '
                               f'Mode: {self.control_mode}, '
                               f'E-Stop: {self.emergency_stopped}')
        self.get_logger().info(f'Joystick: AN1={an1_norm:+.2f} AN2={an2_norm:+.2f} AN3={an3_norm:+.2f} AN4={an4_norm:+.2f}')
        self.get_logger().info(f'Position: X={self.current_positions["x"]:.2f} Y={self.current_positions["y"]:.2f} '
                               f'Z={self.current_positions["z"]:.2f} Yaw={self.current_positions["yaw"]:.1f}deg')
        self.get_logger().info(f'Brake: {"Released" if self.brake_released else "Locked"}')
    
    def destroy_node(self):
        """노드 종료"""
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
            self.get_logger().info('CAN3 bus closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IronMDTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
