#!/usr/bin/env python3
"""
모터 ID 4 (0x144) 테스트 노드
RMD CAN 프로토콜 V4.3에 따른 완전한 모터 제어 테스트

지원하는 명령어:
- 0x77: 시스템 브레이크 릴리즈
- 0x78: 시스템 브레이크 락  
- 0x80: 모터 셧다운
- 0x81: 모터 스탑
- 0xA8: Incremental Position Control

테스트 옵션:
1. 완전한 제어 시퀀스: 브레이크 릴리즈 → 위치 제어 → 브레이크 락
2. 간단한 위치 제어: 브레이크 릴리즈 없이 위치 제어만
"""

import time
import struct
from can_manager import CANManager


class Motor0x144Test:
    """모터 ID 4 (0x144) 테스트 클래스"""

    def __init__(self, can_interface='can2'):
        """
        Args:
            can_interface: CAN 인터페이스 이름
        """
        self.motor_id = 4  # 모터 ID
        self.send_id = 0x140 + self.motor_id  # 0x144
        self.receive_id = 0x240 + self.motor_id  # 0x244

        self.can_manager = CANManager(can_interface)

        # 제어 파라미터
        self.max_speed = 500  # dps (degrees per second)

    def connect(self):
        """CAN 연결"""
        if self.can_manager.connect():
            print(f"CAN 인터페이스 '{self.can_manager.interface}' 연결 성공")
            return True
        else:
            print(f"CAN 인터페이스 '{self.can_manager.interface}' 연결 실패")
            return False

    def disconnect(self):
        """CAN 연결 해제"""
        self.can_manager.disconnect()
        print("CAN 연결 해제")

    def send_brake_release_command(self):
        """
        0x77 명령: 시스템 브레이크 릴리즈
        
        Returns:
            응답 데이터 또는 None
        """
        # 브레이크 릴리즈 명령 데이터 (프로토콜 문서 기준)
        data = struct.pack('<BBBBBBBB',
                          0x77,  # 브레이크 릴리즈 명령
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # 나머지 바이트는 NULL
        )

        print(f"\n브레이크 릴리즈 명령 전송:")
        print(f"  CAN ID: 0x{self.send_id:03X}")
        print(f"  데이터: {data.hex().upper()}")

        # 명령 전송 및 응답 대기
        response = self.can_manager.send_and_receive(
            self.send_id,
            data,
            self.receive_id,
            timeout=2.0
        )

        if response:
            print(f"  브레이크 릴리즈 응답 수신: {response.hex().upper()}")
            return response
        else:
            print("  브레이크 릴리즈 응답 없음 (타임아웃)")
            return None

    def send_brake_lock_command(self):
        """
        0x78 명령: 시스템 브레이크 락
        
        Returns:
            응답 데이터 또는 None
        """
        # 브레이크 락 명령 데이터 (프로토콜 문서 기준)
        data = struct.pack('<BBBBBBBB',
                          0x78,  # 브레이크 락 명령
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # 나머지 바이트는 NULL
        )

        print(f"\n브레이크 락 명령 전송:")
        print(f"  CAN ID: 0x{self.send_id:03X}")
        print(f"  데이터: {data.hex().upper()}")

        # 명령 전송 및 응답 대기
        response = self.can_manager.send_and_receive(
            self.send_id,
            data,
            self.receive_id,
            timeout=2.0
        )

        if response:
            print(f"  브레이크 락 응답 수신: {response.hex().upper()}")
            return response
        else:
            print("  브레이크 락 응답 없음 (타임아웃)")
            return None

    def send_motor_shutdown_command(self):
        """
        0x80 명령: 모터 셧다운 (모터 비활성화)
        
        Returns:
            응답 데이터 또는 None
        """
        # 모터 셧다운 명령 데이터 (프로토콜 문서 기준)
        data = struct.pack('<BBBBBBBB',
                          0x80,  # 모터 셧다운 명령
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # 나머지 바이트는 NULL
        )

        print(f"\n모터 셧다운 명령 전송:")
        print(f"  CAN ID: 0x{self.send_id:03X}")
        print(f"  데이터: {data.hex().upper()}")

        # 명령 전송 및 응답 대기
        response = self.can_manager.send_and_receive(
            self.send_id,
            data,
            self.receive_id,
            timeout=2.0
        )

        if response:
            print(f"  모터 셧다운 응답 수신: {response.hex().upper()}")
            return response
        else:
            print("  모터 셧다운 응답 없음 (타임아웃)")
            return None

    def send_motor_stop_command(self):
        """
        0x81 명령: 모터 스탑 (모터 정지)
        
        Returns:
            응답 데이터 또는 None
        """
        # 모터 스탑 명령 데이터 (프로토콜 문서 기준)
        data = struct.pack('<BBBBBBBB',
                          0x81,  # 모터 스탑 명령
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # 나머지 바이트는 NULL
        )

        print(f"\n모터 스탑 명령 전송:")
        print(f"  CAN ID: 0x{self.send_id:03X}")
        print(f"  데이터: {data.hex().upper()}")

        # 명령 전송 및 응답 대기
        response = self.can_manager.send_and_receive(
            self.send_id,
            data,
            self.receive_id,
            timeout=2.0
        )

        if response:
            print(f"  모터 스탑 응답 수신: {response.hex().upper()}")
            return response
        else:
            print("  모터 스탑 응답 없음 (타임아웃)")
            return None

    def send_incremental_position_command(self, angle_degrees, max_speed_dps):
        """
        0xA8 명령: Incremental Position Control

        Args:
            angle_degrees: 이동할 각도 (degree) - 현재 위치 기준 상대 각도
            max_speed_dps: 최대 속도 (degrees per second)

        Returns:
            응답 데이터 또는 None
        """
        # 각도를 프로토콜 단위로 변환 (0.01 degree/LSB)
        angle_control = int(angle_degrees * 100)

        # int32_t로 변환 (부호 있는 32비트 정수)
        if angle_control < 0:
            angle_control = (1 << 32) + angle_control

        # 데이터 패킷 생성
        # DATA[0] = 0xA8 (명령 바이트)
        # DATA[1] = 0x00 (NULL)
        # DATA[2:3] = maxSpeed (uint16_t, little-endian)
        # DATA[4:7] = angleControl (int32_t, little-endian)
        data = struct.pack('<BB H I',
                          0xA8,  # 명령 바이트
                          0x00,  # NULL
                          max_speed_dps,  # 최대 속도 (uint16_t)
                          angle_control & 0xFFFFFFFF  # 각도 제어 (int32_t)
        )

        print(f"\n명령 전송:")
        print(f"  CAN ID: 0x{self.send_id:03X}")
        print(f"  목표 각도: {angle_degrees}도 (상대 위치)")
        print(f"  최대 속도: {max_speed_dps} dps")
        print(f"  데이터: {data.hex().upper()}")

        # 명령 전송 및 응답 대기
        response = self.can_manager.send_and_receive(
            self.send_id,
            data,
            self.receive_id,
            timeout=2.0
        )

        if response:
            self.parse_response(response)
            return response
        else:
            print("  응답 없음 (타임아웃)")
            return None

    def parse_response(self, data):
        """
        응답 데이터 파싱

        Args:
            data: 응답 데이터 (8바이트)
        """
        if len(data) < 8:
            print(f"  응답 데이터 길이 부족: {len(data)}바이트")
            return

        # 데이터 파싱
        # DATA[0] = 명령 바이트 (0xA8)
        # DATA[1] = 모터 온도 (int8_t, 1°C/LSB)
        # DATA[2:3] = 토크 전류 (int16_t, 0.01A/LSB)
        # DATA[4:5] = 모터 속도 (int16_t, 1dps/LSB)
        # DATA[6:7] = 모터 각도 (int16_t, 1degree/LSB)

        cmd_byte = data[0]
        temperature = struct.unpack('b', data[1:2])[0]  # signed int8
        torque_current_raw = struct.unpack('<h', data[2:4])[0]  # signed int16
        speed_raw = struct.unpack('<h', data[4:6])[0]  # signed int16
        angle_raw = struct.unpack('<h', data[6:8])[0]  # signed int16

        # 실제 값으로 변환
        torque_current = torque_current_raw * 0.01  # A
        speed = speed_raw  # dps
        angle = angle_raw  # degree

        print(f"\n응답 수신:")
        print(f"  CAN ID: 0x{self.receive_id:03X}")
        print(f"  명령: 0x{cmd_byte:02X}")
        print(f"  온도: {temperature}°C")
        print(f"  토크 전류: {torque_current:.2f}A")
        print(f"  속도: {speed} dps")
        print(f"  각도: {angle}° (제로 위치 기준)")
        print(f"  원본 데이터: {data.hex().upper()}")

    def test_rotation(self):
        """
        브레이크 릴리즈 → 위치 제어 → 브레이크 락 순서로 테스트
        """
        print("\n" + "="*60)
        print("모터 ID 4 (0x144) - 완전한 제어 시퀀스 테스트")
        print("="*60)

        # 1. 브레이크 릴리즈
        print("\n[단계 1] 브레이크 릴리즈")
        print("-" * 60)
        brake_response = self.send_brake_release_command()
        if not brake_response:
            print("브레이크 릴리즈 실패")
            return False
        
        # 브레이크 릴리즈 후 잠시 대기
        print("\n브레이크 릴리즈 완료 대기 중... (1초)")
        time.sleep(1.0)

        # 2. +15도 회전
        print("\n[단계 2] 현재 위치에서 +15도 회전")
        print("-" * 60)
        response1 = self.send_incremental_position_command(
            angle_degrees=15.0,
            max_speed_dps=self.max_speed
        )

        if not response1:
            print("첫 번째 위치 제어 명령 실패")
            return False

        # 동작 완료 대기 (15도 / 500dps = 0.03초 + 여유)
        wait_time = 2.0
        print(f"\n동작 완료 대기 중... ({wait_time}초)")
        time.sleep(wait_time)

        # 3. -15도 회전 (원위치로 복귀)
        print("\n[단계 3] 현재 위치에서 -15도 회전 (원위치 복귀)")
        print("-" * 60)
        response2 = self.send_incremental_position_command(
            angle_degrees=-15.0,
            max_speed_dps=self.max_speed
        )

        if not response2:
            print("두 번째 위치 제어 명령 실패")
            return False

        # 동작 완료 대기
        print(f"\n동작 완료 대기 중... ({wait_time}초)")
        time.sleep(wait_time)

        # 4. 모터 스탑
        print("\n[단계 4] 모터 스탑")
        print("-" * 60)
        stop_response = self.send_motor_stop_command()
        if not stop_response:
            print("모터 스탑 실패")
            return False

        # 5. 브레이크 락 (선택사항)
        print("\n[단계 5] 브레이크 락 (안전을 위해)")
        print("-" * 60)
        lock_response = self.send_brake_lock_command()
        if not lock_response:
            print("브레이크 락 실패")
            return False

        print("\n" + "="*60)
        print("완전한 제어 시퀀스 테스트 완료!")
        print("="*60)

        return True

    def test_simple_rotation(self):
        """
        간단한 회전 테스트 (브레이크 릴리즈 없이)
        """
        print("\n" + "="*60)
        print("모터 ID 4 (0x144) - 간단한 위치 제어 테스트")
        print("="*60)

        # +15도 회전
        print("\n[테스트 1] 현재 위치에서 +15도 회전")
        print("-" * 60)
        response1 = self.send_incremental_position_command(
            angle_degrees=15.0,
            max_speed_dps=self.max_speed
        )

        if not response1:
            print("첫 번째 명령 실패")
            return False

        # 동작 완료 대기
        wait_time = 2.0
        print(f"\n동작 완료 대기 중... ({wait_time}초)")
        time.sleep(wait_time)

        # -15도 회전 (원위치로 복귀)
        print("\n[테스트 2] 현재 위치에서 -15도 회전 (원위치 복귀)")
        print("-" * 60)
        response2 = self.send_incremental_position_command(
            angle_degrees=-15.0,
            max_speed_dps=self.max_speed
        )

        if not response2:
            print("두 번째 명령 실패")
            return False

        # 동작 완료 대기
        print(f"\n동작 완료 대기 중... ({wait_time}초)")
        time.sleep(wait_time)

        print("\n" + "="*60)
        print("간단한 테스트 완료!")
        print("="*60)

        return True


def main():
    """메인 함수"""
    # 모터 테스트 객체 생성
    motor_test = Motor0x144Test(can_interface='can2')

    # CAN 연결
    if not motor_test.connect():
        print("CAN 연결 실패. 프로그램 종료.")
        return

    try:
        # 테스트 옵션 선택
        print("\n테스트 옵션을 선택하세요:")
        print("1. 완전한 제어 시퀀스 테스트 (브레이크 릴리즈 → 위치 제어 → 브레이크 락)")
        print("2. 간단한 위치 제어 테스트 (브레이크 릴리즈 없이)")
        
        choice = input("\n선택 (1 또는 2, 기본값: 1): ").strip()
        
        if choice == "2":
            print("\n간단한 위치 제어 테스트를 실행합니다...")
            motor_test.test_simple_rotation()
        else:
            print("\n완전한 제어 시퀀스 테스트를 실행합니다...")
            motor_test.test_rotation()

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단됨")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 연결 해제
        motor_test.disconnect()


if __name__ == '__main__':
    main()
