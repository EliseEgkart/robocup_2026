#!/usr/bin/env python3
#=====================================================#
# 기능: 키 입력 즉시 반영 + 누르는 동안만 동작
# - W/S/A/D: 전진/후진/좌/우 회전
# - X: 정지
# - Q: 종료
#=====================================================#

import sys
import termios
import tty
import select
import time
from motor_driver import MotorDriver


def get_key(timeout=0.05):
    """논블로킹 키 입력: timeout 내 입력 없으면 None 반환"""
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None


if __name__ == "__main__":
    md = MotorDriver()

    speed = 40
    turn_speed = 20
    stop_time = 0.15  # 키 떼면 이 시간 후 정지
    last_cmd_time = time.time()
    current_cmd = (0, 0)

    # 터미널 설정
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    print("키보드 실시간 제어 시작")
    print("W: 전진 | S: 후진 | A: 좌회전 | D: 우회전 | X: 정지 | Q: 종료")

    try:
        while True:
            key = get_key()

            if key:
                key = key.lower()
                if key == 'w':
                    current_cmd = (speed, speed)
                    md.send_vel_cmd(*current_cmd)
                    last_cmd_time = time.time()
                    print("전진")

                elif key == 's':
                    current_cmd = (-speed, -speed)
                    md.send_vel_cmd(*current_cmd)
                    last_cmd_time = time.time()
                    print("후진")

                elif key == 'a':
                    current_cmd = (-turn_speed, turn_speed)
                    md.send_vel_cmd(*current_cmd)
                    last_cmd_time = time.time()
                    print("좌회전")

                elif key == 'd':
                    current_cmd = (turn_speed, -turn_speed)
                    md.send_vel_cmd(*current_cmd)
                    last_cmd_time = time.time()
                    print("우회전")

                elif key == 'x':
                    current_cmd = (0, 0)
                    md.send_vel_cmd(*current_cmd)
                    print("정지")

                elif key == 'q':
                    md.send_vel_cmd(0, 0)
                    print("종료합니다.")
                    break

            # 키 입력이 일정 시간 없으면 자동 정지
            if time.time() - last_cmd_time > stop_time and current_cmd != (0, 0):
                current_cmd = (0, 0)
                md.send_vel_cmd(0, 0)
                print("자동 정지")

            time.sleep(0.01)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        md.send_vel_cmd(0, 0)
        print("프로그램 종료, 모터 정지 완료.")
