#!/usr/bin/env python3
"""
CAN 연결 및 모터 응답 테스트
간단하게 0x9C 명령을 보내고 응답을 확인
"""

import sys
import time
sys.path.insert(0, '/home/test/ros2_ws/src/rmd_robot_control')

from rmd_robot_control.can_manager import CANManager
import struct


def test_motor_connection(motor_id=0x144):
    """모터 연결 및 응답 테스트"""
    print(f"\n{'='*60}")
    print(f"모터 0x{motor_id:03X} 연결 테스트")
    print(f"{'='*60}\n")

    # CAN 매니저 생성
    can_manager = CANManager('can2')

    # CAN 연결
    if not can_manager.connect():
        print("❌ CAN 연결 실패")
        return False

    print("✓ CAN 연결 성공\n")

    # 0x9C 명령 (READ_MOTOR_STATUS) 전송
    status_cmd = bytes([0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    print(f"📤 0x9C 명령 전송 (모터 상태 읽기)...")
    print(f"   CAN ID: 0x{motor_id:03X}")
    print(f"   데이터: {status_cmd.hex().upper()}\n")

    # 응답 ID 계산
    motor_number = motor_id - 0x140
    response_id = 0x240 + motor_number

    print(f"⏳ 응답 대기 (예상 응답 ID: 0x{response_id:03X})...\n")

    # 명령 전송
    can_manager.send_frame(motor_id, status_cmd)

    # 응답 대기 (최대 2초)
    response = None
    for i in range(20):  # 2초 동안 100ms 간격으로 확인
        frame = can_manager.receive_frame(timeout=0.1)
        if frame:
            can_id, data = frame
            print(f"📥 응답 수신:")
            print(f"   CAN ID: 0x{can_id:03X}")
            print(f"   데이터: {data.hex().upper()}")

            if can_id == response_id:
                response = data
                break
        time.sleep(0.1)

    if not response:
        print("\n❌ 응답 없음 (타임아웃)")
        print("\n가능한 원인:")
        print("1. 모터가 연결되어 있지 않음")
        print("2. 모터 ID가 올바르지 않음")
        print("3. CAN 버스 속도가 일치하지 않음")
        print("4. 모터 전원이 꺼져 있음")
        can_manager.disconnect()
        return False

    # 응답 파싱
    print(f"\n✓ 응답 수신 성공!\n")

    if len(response) >= 8:
        command = response[0]
        temperature = struct.unpack('<b', response[1:2])[0]
        current = struct.unpack('<h', response[2:4])[0] * 0.01
        speed = struct.unpack('<h', response[4:6])[0]
        angle = struct.unpack('<h', response[6:8])[0]

        print(f"파싱 결과:")
        print(f"  명령 코드: 0x{command:02X}")
        print(f"  온도: {temperature}°C")
        print(f"  토크 전류: {current:.2f}A")
        print(f"  속도: {speed} dps")
        print(f"  각도: {angle}° (제로 위치 기준)\n")

    can_manager.disconnect()
    print(f"{'='*60}")
    print("테스트 완료!")
    print(f"{'='*60}\n")

    return True


def main():
    """메인 함수"""
    # 기본 모터 ID: 0x144 (관절 2)
    motor_id = 0x144

    # 명령행 인수로 모터 ID 지정 가능
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1], 16)
        except ValueError:
            print(f"잘못된 모터 ID: {sys.argv[1]}")
            print("사용법: python3 test_can_connection.py [모터ID(16진수)]")
            print("예: python3 test_can_connection.py 0x144")
            return

    test_motor_connection(motor_id)


if __name__ == '__main__':
    main()
