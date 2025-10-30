#!/usr/bin/env python3
"""
모터 테스트 노드
개별 모터 테스트 및 디버깅용
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class MotorTestNode(Node):
    """모터 테스트 노드"""
    
    def __init__(self):
        super().__init__('motor_test_node')
        
        # 테스트 모드 선택
        self.declare_parameter('test_mode', 'cmd_vel')  # 'cmd_vel', 'position', 'individual'
        self.test_mode = self.get_parameter('test_mode').value
        
        # CMD_VEL 테스트용 퍼블리셔
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 위치제어 테스트용 퍼블리셔
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )
        
        # 개별 관절 제어용 퍼블리셔들
        self.joint_publishers = []
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        
        for joint_name in joint_names:
            publisher = self.create_publisher(
                Float64MultiArray,
                f'/{joint_name}/position',
                10
            )
            self.joint_publishers.append(publisher)
        
        # 테스트 타이머
        self.test_timer = self.create_timer(5.0, self.run_test)
        self.test_step = 0
        
        self.get_logger().info(f"모터 테스트 노드 시작 - 모드: {self.test_mode}")
    
    def run_test(self):
        """테스트 실행"""
        if self.test_mode == 'cmd_vel':
            self.test_cmd_vel()
        elif self.test_mode == 'position':
            self.test_position_control()
        elif self.test_mode == 'individual':
            self.test_individual_joints()
        
        self.test_step += 1
    
    def test_cmd_vel(self):
        """CMD_VEL 테스트"""
        cmd_vel = Twist()
        
        if self.test_step % 4 == 0:
            cmd_vel.linear.x = 0.5  # 전진
            self.get_logger().info("CMD_VEL 테스트: 전진")
        elif self.test_step % 4 == 1:
            cmd_vel.linear.x = -0.5  # 후진
            self.get_logger().info("CMD_VEL 테스트: 후진")
        elif self.test_step % 4 == 2:
            cmd_vel.angular.z = 0.5  # 좌회전
            self.get_logger().info("CMD_VEL 테스트: 좌회전")
        elif self.test_step % 4 == 3:
            cmd_vel.angular.z = -0.5  # 우회전
            self.get_logger().info("CMD_VEL 테스트: 우회전")
        
        self.cmd_vel_publisher.publish(cmd_vel)
    
    def test_position_control(self):
        """위치제어 테스트"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_time().to_msg()
        trajectory.header.frame_id = 'base_link'
        
        trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        
        point = JointTrajectoryPoint()
        
        if self.test_step % 3 == 0:
            # 모든 관절을 0도로
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("위치제어 테스트: 모든 관절 0도")
        elif self.test_step % 3 == 1:
            # 모든 관절을 90도로
            point.positions = [1.57, 1.57, 1.57, 1.57, 1.57]  # 90도 = π/2 라디안
            self.get_logger().info("위치제어 테스트: 모든 관절 90도")
        elif self.test_step % 3 == 2:
            # 모든 관절을 -90도로
            point.positions = [-1.57, -1.57, -1.57, -1.57, -1.57]  # -90도
            self.get_logger().info("위치제어 테스트: 모든 관절 -90도")
        
        point.time_from_start.sec = 2
        trajectory.points = [point]
        
        self.trajectory_publisher.publish(trajectory)
    
    def test_individual_joints(self):
        """개별 관절 테스트"""
        joint_index = self.test_step % len(self.joint_publishers)
        joint_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'][joint_index]
        
        position_msg = Float64MultiArray()
        
        if self.test_step % 6 == 0:
            position_msg.data = [0.0]  # 0도
            self.get_logger().info(f"개별 관절 테스트: {joint_name} -> 0도")
        elif self.test_step % 6 == 1:
            position_msg.data = [1.57]  # 90도
            self.get_logger().info(f"개별 관절 테스트: {joint_name} -> 90도")
        elif self.test_step % 6 == 2:
            position_msg.data = [-1.57]  # -90도
            self.get_logger().info(f"개별 관절 테스트: {joint_name} -> -90도")
        elif self.test_step % 6 == 3:
            position_msg.data = [3.14]  # 180도
            self.get_logger().info(f"개별 관절 테스트: {joint_name} -> 180도")
        elif self.test_step % 6 == 4:
            position_msg.data = [-3.14]  # -180도
            self.get_logger().info(f"개별 관절 테스트: {joint_name} -> -180도")
        elif self.test_step % 6 == 5:
            position_msg.data = [0.0]  # 원점 복귀
            self.get_logger().info(f"개별 관절 테스트: {joint_name} -> 원점 복귀")
        
        self.joint_publishers[joint_index].publish(position_msg)


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = MotorTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



