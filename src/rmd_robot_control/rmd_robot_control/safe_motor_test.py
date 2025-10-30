#!/usr/bin/env python3
"""
안전한 모터 통신 테스트 노드
모터를 실제로 움직이지 않고 CAN 통신만 테스트
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import struct
from typing import Dict, List

from .can_manager import CANManager


class SafeMotorTestNode(Node):
    """안전한 모터 테스트 노드"""
    
    def __init__(self):
        super().__init__('safe_motor_test_node')
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('test_mode', 'communication')  # 'communication', 'status_read', 'id_check'
        self.declare_parameter('motor_ids', [0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 0x147])
        
        # 파라미터 가져오기
        self.can_interface = self.get_parameter('can_interface').value
        self.test_mode = self.get_parameter('test_mode').value
        self.motor_ids = self.get_parameter('motor_ids').value
        
        # CAN 매니저 초기화
        self.can_manager = CANManager(self.can_interface)
        
        # 모터 상태 저장
        self.motor_states: Dict[int, Dict] = {}
        self.communication_success: Dict[int, bool] = {}
        
        for motor_id in self.motor_ids:
            self.motor_states[motor_id] = {
                'position': 0.0,
                'velocity': 0.0,
                'torque': 0.0,
                'temperature': 0.0,
                'voltage': 0.0,
                'last_response_time': 0.0
            }
            self.communication_success[motor_id] = False
        
        # RMD-X4 명령 코드 (읽기 전용)
        self.COMMANDS = {
            'READ_MOTOR_STATUS': 0x9C,
            'READ_MOTOR_POSITION': 0x90,
            'READ_MOTOR_VELOCITY': 0x92,
            'READ_MOTOR_TORQUE': 0x91,
            'READ_MOTOR_ERROR': 0x93,
            'READ_MOTOR_TEMPERATURE': 0x94,
            'READ_MOTOR_VOLTAGE': 0x95,
            'READ_MOTOR_CURRENT': 0x96,
        }
        
        # 상태 발행용 퍼블리셔
        self.motor_status_publisher = self.create_publisher(
            Float64MultiArray,
            '/safe_test/motor_status',
            10
        )
        
        # CAN 연결 및 콜백 등록
        if self.can_manager.connect():
            self.get_logger().info(f"CAN 인터페이스 {self.can_interface} 연결 성공")
            
            # 모든 모터 응답 콜백 등록
            for motor_id in self.motor_ids:
                response_id = motor_id + 0x100  # 응답 ID = 명령 ID + 0x100
                self.can_manager.register_callback(
                    response_id,
                    lambda can_id, data, mid=motor_id: self.motor_response_callback(mid, data)
                )
            
            # 백그라운드 수신 스레드 시작
            self.can_manager.start_receive_thread()
            
            self.get_logger().info("안전한 모터 테스트 노드 시작")
            self.get_logger().warning("⚠️  모터는 움직이지 않습니다. 통신 테스트만 수행합니다.")
        else:
            self.get_logger().error(f"CAN 인터페이스 {self.can_interface} 연결 실패")
            return
        
        # 테스트 타이머 설정
        self.test_timer = self.create_timer(1.0, self.run_safe_test)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        self.test_step = 0
        self.test_results = {}
    
    def run_safe_test(self):
        """안전한 테스트 실행"""
        if self.test_mode == 'communication':
            self.test_communication()
        elif self.test_mode == 'status_read':
            self.test_status_read()
        elif self.test_mode == 'id_check':
            self.test_id_check()
        
        self.test_step += 1
    
    def test_communication(self):
        """통신 테스트"""
        self.get_logger().info(f"=== 통신 테스트 단계 {self.test_step} ===")
        
        for motor_id in self.motor_ids:
            # 상태 읽기 명령 전송 (안전한 읽기 전용 명령)
            status_cmd = bytes([self.COMMANDS['READ_MOTOR_STATUS']]) + b'\x00' * 7
            success = self.can_manager.send_frame(motor_id, status_cmd)
            
            if success:
                self.get_logger().info(f"모터 0x{motor_id:03X}: 상태 읽기 명령 전송 성공")
            else:
                self.get_logger().error(f"모터 0x{motor_id:03X}: 상태 읽기 명령 전송 실패")
    
    def test_status_read(self):
        """상태 읽기 테스트"""
        self.get_logger().info(f"=== 상태 읽기 테스트 단계 {self.test_step} ===")
        
        test_commands = [
            ('READ_MOTOR_STATUS', '모터 상태'),
            ('READ_MOTOR_POSITION', '모터 위치'),
            ('READ_MOTOR_VELOCITY', '모터 속도'),
            ('READ_MOTOR_TORQUE', '모터 토크'),
            ('READ_MOTOR_TEMPERATURE', '모터 온도'),
            ('READ_MOTOR_VOLTAGE', '모터 전압'),
        ]
        
        cmd_name, cmd_desc = test_commands[self.test_step % len(test_commands)]
        command_code = self.COMMANDS[cmd_name]
        
        self.get_logger().info(f"{cmd_desc} 읽기 테스트")
        
        for motor_id in self.motor_ids:
            cmd_data = bytes([command_code]) + b'\x00' * 7
            success = self.can_manager.send_frame(motor_id, cmd_data)
            
            if success:
                self.get_logger().info(f"모터 0x{motor_id:03X}: {cmd_desc} 명령 전송 성공")
            else:
                self.get_logger().error(f"모터 0x{motor_id:03X}: {cmd_desc} 명령 전송 실패")
    
    def test_id_check(self):
        """모터 ID 확인 테스트"""
        self.get_logger().info(f"=== 모터 ID 확인 테스트 단계 {self.test_step} ===")
        
        # 각 모터 ID에 대해 상태 읽기 명령 전송
        for motor_id in self.motor_ids:
            status_cmd = bytes([self.COMMANDS['READ_MOTOR_STATUS']]) + b'\x00' * 7
            success = self.can_manager.send_frame(motor_id, status_cmd)
            
            if success:
                self.get_logger().info(f"모터 ID 0x{motor_id:03X}: 명령 전송 성공")
            else:
                self.get_logger().error(f"모터 ID 0x{motor_id:03X}: 명령 전송 실패")
        
        # 통신 성공률 계산
        total_motors = len(self.motor_ids)
        successful_motors = sum(1 for success in self.communication_success.values() if success)
        success_rate = (successful_motors / total_motors) * 100
        
        self.get_logger().info(f"통신 성공률: {success_rate:.1f}% ({successful_motors}/{total_motors})")
    
    def motor_response_callback(self, motor_id: int, data: bytes):
        """모터 응답 콜백"""
        if len(data) < 8:
            return
        
        command = data[0]
        current_time = time.time()
        
        # 통신 성공 표시
        self.communication_success[motor_id] = True
        self.motor_states[motor_id]['last_response_time'] = current_time
        
        if command == self.COMMANDS['READ_MOTOR_STATUS']:
            # 상태 응답 파싱
            position = struct.unpack('<i', data[1:5])[0] / 100.0  # 0.01도 단위
            velocity = struct.unpack('<h', data[5:7])[0] / 10.0  # 0.1 RPM 단위
            torque = struct.unpack('<h', data[7:9])[0] / 10.0    # 0.1A 단위
            
            self.motor_states[motor_id]['position'] = position
            self.motor_states[motor_id]['velocity'] = velocity
            self.motor_states[motor_id]['torque'] = torque
            
            self.get_logger().info(
                f"모터 0x{motor_id:03X} 상태: 위치={position:.1f}도, "
                f"속도={velocity:.1f}RPM, 토크={torque:.1f}A"
            )
        
        elif command == self.COMMANDS['READ_MOTOR_TEMPERATURE']:
            # 온도 응답 파싱
            temperature = struct.unpack('<h', data[1:3])[0] / 10.0  # 0.1도 단위
            self.motor_states[motor_id]['temperature'] = temperature
            
            self.get_logger().info(f"모터 0x{motor_id:03X} 온도: {temperature:.1f}도")
        
        elif command == self.COMMANDS['READ_MOTOR_VOLTAGE']:
            # 전압 응답 파싱
            voltage = struct.unpack('<h', data[1:3])[0] / 10.0  # 0.1V 단위
            self.motor_states[motor_id]['voltage'] = voltage
            
            self.get_logger().info(f"모터 0x{motor_id:03X} 전압: {voltage:.1f}V")
    
    def publish_status(self):
        """상태 발행"""
        motor_status = Float64MultiArray()
        motor_status.data = []
        
        for motor_id in self.motor_ids:
            motor_status.data.extend([
                self.motor_states[motor_id]['position'],
                self.motor_states[motor_id]['velocity'],
                self.motor_states[motor_id]['torque'],
                self.motor_states[motor_id]['temperature'],
                self.motor_states[motor_id]['voltage'],
                1.0 if self.communication_success[motor_id] else 0.0
            ])
        
        self.motor_status_publisher.publish(motor_status)
    
    def print_test_summary(self):
        """테스트 결과 요약 출력"""
        self.get_logger().info("=== 안전한 모터 테스트 결과 요약 ===")
        
        for motor_id in self.motor_ids:
            success = self.communication_success[motor_id]
            status = "✅ 통신 성공" if success else "❌ 통신 실패"
            
            self.get_logger().info(f"모터 0x{motor_id:03X}: {status}")
            
            if success:
                state = self.motor_states[motor_id]
                self.get_logger().info(
                    f"  - 위치: {state['position']:.1f}도"
                    f"  - 속도: {state['velocity']:.1f}RPM"
                    f"  - 토크: {state['torque']:.1f}A"
                    f"  - 온도: {state['temperature']:.1f}도"
                    f"  - 전압: {state['voltage']:.1f}V"
                )
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.print_test_summary()
        self.can_manager.disconnect()
        super().destroy_node()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = SafeMotorTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



