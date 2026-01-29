#!/usr/bin/env python3
#=====================================================#
# 기능: ROS2 조이스틱 → MotorDriver 제어 노드
#=====================================================#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from serial_comm.motor_driver import MotorDriver

class MDTeleop(Node):
    def __init__(self):
        super().__init__('md_teleop_node')
        self.get_logger().info("Joystick Teleop → MotorDriver Node 시작")

        # 모터드라이버 초기화
        self.md = MotorDriver()

        # 조이스틱 구독
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # 파라미터
        self.declare_parameter('max_speed', 400)
        self.max_speed = self.get_parameter('max_speed').value

    def joy_callback(self, msg: Joy):
        # 조이스틱 축 매핑 (Xbox/Logitech 기준)
        #   left stick vertical (Y): msg.axes[1]
        #   right stick horizontal (X): msg.axes[3]
        linear = msg.axes[1]   # 전/후
        angular = msg.axes[2]  # 좌/우

        # 속도 계산 (차동 제어)
        vL = int(self.max_speed * (linear - angular))
        vR = int(self.max_speed * (linear + angular))

        self.md.send_vel_cmd(vL, vR)
        self.get_logger().info(f"cmd: L={vL}, R={vR}")

def main(args=None):
    rclpy.init(args=args)
    node = MDTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
