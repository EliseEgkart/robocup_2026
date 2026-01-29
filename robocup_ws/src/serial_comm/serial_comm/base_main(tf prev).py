import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
import threading
import time

from amr_msgs.msg import WheelMotor
from serial_comm.motor_driver import MotorDriver


class Nodelet(Node):
    def __init__(self):
        super().__init__('base_main')

        # -------------------------
        # Publishers / Subscribers
        # -------------------------
        self.pub = self.create_publisher(WheelMotor, '/wheelmotor', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 중요: robot_state_publisher 표준 토픽
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 50)

        self.amr_data_distance = self.create_publisher(String, '/amr_data_distance', 10)

        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 100)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 100)

        # -------------------------
        # Timing
        # -------------------------
        self.dt = 0.02  # 50 Hz
        self.timer_ = self.create_timer(self.dt, self.timer_callback)

        # -------------------------
        # TF
        # -------------------------
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------------
        # Motor Driver (I/O 분리)
        # -------------------------
        self.md = MotorDriver()

        # I/O 스레드 공유 데이터 (락으로 보호)
        self._lock = threading.Lock()
        self._running = True

        # "마지막으로 수신된" 모터 상태값 (timer_callback은 이것만 사용)
        self._md_pos1 = 0.0
        self._md_pos2 = 0.0
        self._md_rpm1 = 0.0
        self._md_rpm2 = 0.0
        self._md_cur1 = 0.0
        self._md_cur2 = 0.0
        self._md_last_rx_time = self.get_clock().now()

        # I/O 스레드 시작
        self._io_thread = threading.Thread(target=self._io_loop, daemon=True)
        self._io_thread.start()

        # -------------------------
        # Control / State
        # -------------------------
        self.loopcnt = 0
        self.firstloop = True
        self.JOY_CONTROL = False

        # PID related variables (원본 유지)
        self.p_gain = 1.
        self.i_gain = 0.
        self.d_gain = 0.01
        self.forget = 0.99

        self.err1_prev, self.err1_i = 0., 0.
        self.err2_prev, self.err2_i = 0., 0.
        self.velocity1, self.velocity2 = 0, 0

        # target position
        self.target_pos1, self.target_pos2 = 0.0, 0.0

        # joy
        self.joy_fb = 0.0
        self.joy_lr = 0.0
        self.v_gain = 100
        self.w_gain = 50
        self.joy_r2 = 0.0
        self.joy_l2 = 0.0
        self.change_mode = 0
        self.joy_stop = 0

        self.joy_speed_up = 0
        self.joy_speed_down = 0
        self.gain_count = 2
        self.gain_list = [0.5, 0.7, 1.0, 1.3, 1.5, 1.7, 2.0, 2.3, 2.5, 4.0]
        self.joy_speed_up_old = 0
        self.joy_speed_down_old = 0

        self.msg_wheelmotor = WheelMotor()

        # odom param
        self.wheel_separation = 0.32
        self.wheel_diameter = 0.13
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.last_time = self.get_clock().now()

        # accumulated_distance
        self.accumulated_distance = 0.0
        self.amr_data_distance_msg = String()

        # -------------------------
        # JointState: "delta 적분" 누적 각도
        # -------------------------
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # 엔코더 누적값 기준점(첫 수신 시 고정)
        self.enc0_set = False
        self.enc0_pos1 = 0.0
        self.enc0_pos2 = 0.0

        # 이전 엔코더(증분 계산)
        self.prev_enc1 = None
        self.prev_enc2 = None

        # -------------------------
        # Marker 관련(원본 유지 - 여기서는 생략)
        # -------------------------
        self.marker_detected = False
        self.marker_detected_count = 0
        self.marker_deadreckonmode = False
        self.marker_target_distance = 0.9
        self.marker_target_distance2 = 0.9
        self.marker_moving_distance = 0.0
        self.marker_moving_distance2 = 0.0
        self.target_marker_id = 3
        self.next_marker_id = 7
        self.marker_rotation_en_count = 0.0
        self.marker7_1 = True

    # -------------------------
    # Background I/O loop
    # -------------------------
    def _io_loop(self):
        """
        recv_motor_state()가 블로킹/지연돼도 timer_callback을 막지 않게 분리.
        """
        while self._running and rclpy.ok():
            try:
                self.md.recv_motor_state()

                with self._lock:
                    self._md_pos1 = float(self.md.pos1)
                    self._md_pos2 = float(self.md.pos2)
                    self._md_rpm1 = float(self.md.rpm1)
                    self._md_rpm2 = float(self.md.rpm2)
                    self._md_cur1 = float(self.md.current1)
                    self._md_cur2 = float(self.md.current2)
                    self._md_last_rx_time = self.get_clock().now()

            except Exception as e:
                # I/O 에러가 있어도 메인 루프가 죽지 않게
                # 필요하면 로그를 더 강하게 찍어도 됨
                self.get_logger().warn(f"[io_loop] recv_motor_state failed: {e}")

            # 너무 바쁘게 돌지 않게 약간 쉼 (환경에 맞게 조절)
            time.sleep(0.001)

    def stop(self):
        self._running = False
        try:
            if self._io_thread.is_alive():
                self._io_thread.join(timeout=0.5)
        except Exception:
            pass

    # -------------------------
    # Main 50Hz loop (절대 블로킹 금지)
    # -------------------------
    def timer_callback(self):
        self.loopcnt += 1

        # 마지막 수신값 스냅샷
        with self._lock:
            pos1 = self._md_pos1
            pos2 = self._md_pos2
            rpm1 = self._md_rpm1
            rpm2 = self._md_rpm2
            cur1 = self._md_cur1
            cur2 = self._md_cur2

        # 첫 루프: 엔코더 기준점 잡기
        if self.firstloop:
            # 명령은 메인 스레드에서만 송신
            self.md.send_vel_cmd(self.velocity1, self.velocity2)

            self.target_pos1 = pos1
            self.target_pos2 = pos2

            self.enc0_pos1 = pos1
            self.enc0_pos2 = pos2
            self.enc0_set = True

            self.prev_enc1 = pos1
            self.prev_enc2 = pos2

            self.firstloop = False
            return

        # -------------------------
        # Control (원본 로직 유지)
        # -------------------------
        if self.JOY_CONTROL:
            vel1 = self.v_gain * self.joy_fb - self.w_gain * self.joy_lr
            vel2 = self.v_gain * self.joy_fb + self.w_gain * self.joy_lr

            if self.joy_stop == 1:
                self.md.send_position_cmd(int(pos1), int(pos2), int(60), int(60))
            else:
                self.md.send_vel_cmd(vel1, vel2)

            self.msg_wheelmotor.target1 = int(vel1)
            self.msg_wheelmotor.target2 = int(vel2)
        else:
            # 여기 부분은 네 deadreckon 로직 그대로 유지 가능
            # (생략 없이 최소한의 형태로 남김)
            self.md.send_position_cmd(int(self.target_pos1), int(self.target_pos2), int(60), int(60))
            self.msg_wheelmotor.target1 = int(self.target_pos1)
            self.msg_wheelmotor.target2 = int(self.target_pos2)

        # -------------------------
        # WheelMotor publish (수신 스냅샷 기반)
        # -------------------------
        self.msg_wheelmotor.position1 = int(pos1)
        self.msg_wheelmotor.position2 = int(pos2)
        self.msg_wheelmotor.velocity1 = float(rpm1)
        self.msg_wheelmotor.velocity2 = float(rpm2)
        self.msg_wheelmotor.current1 = int(cur1)
        self.msg_wheelmotor.current2 = int(cur2)

        # (원본 부호 정리 유지)
        rpm_left = -rpm1
        rpm_right = rpm2
        self.msg_wheelmotor.v_x = (rpm_left + rpm_right) * np.pi * self.wheel_diameter / (60.0 * 2.0)
        self.msg_wheelmotor.w_z = -(rpm_right - rpm_left) * np.pi * self.wheel_diameter / (60.0 * self.wheel_separation)

        self.pub.publish(self.msg_wheelmotor)

        # -------------------------
        # JointState (delta 적분 누적)
        # -------------------------
        if self.prev_enc1 is None:
            self.prev_enc1 = pos1
            self.prev_enc2 = pos2

        d1 = pos1 - self.prev_enc1
        d2 = pos2 - self.prev_enc2
        self.prev_enc1 = pos1
        self.prev_enc2 = pos2

        # 왼쪽 바퀴 부호 반전(원본 의도 유지)
        d_left = -d1
        d_right = d2

        # encoder_gain: counts per revolution 가정
        # counts -> rad
        self.left_wheel_angle += 2.0 * np.pi * (d_left / float(self.md.encoder_gain))
        self.right_wheel_angle += 2.0 * np.pi * (d_right / float(self.md.encoder_gain))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_left_joint', 'wheel_right_joint']  # URDF와 반드시 일치
        js.position = [self.left_wheel_angle, self.right_wheel_angle]

        # velocity도 넣어주면 RViz/상태추정이 더 안정적일 때가 있음(선택)
        # rpm -> rad/s
        js.velocity = [
            2.0 * np.pi * (rpm_left / 60.0),
            2.0 * np.pi * (rpm_right / 60.0)
        ]

        self.joint_state_pub.publish(js)

        # -------------------------
        # Odom (delta 기반)
        # -------------------------
        # wheel displacement (m)
        left_wheel_disp = (d_left / float(self.md.encoder_gain)) * (np.pi * self.wheel_diameter)
        right_wheel_disp = (d_right / float(self.md.encoder_gain)) * (np.pi * self.wheel_diameter)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # dt 방어
        if dt <= 1e-6:
            dt = self.dt

        linear_velocity = -(left_wheel_disp + right_wheel_disp) / (2.0 * dt)
        angular_velocity = -(right_wheel_disp - left_wheel_disp) / (self.wheel_separation * dt)

        self.pose_x += linear_velocity * np.cos(self.pose_theta) * dt
        self.pose_y += linear_velocity * np.sin(self.pose_theta) * dt
        self.pose_theta += angular_velocity * dt

        if self.pose_theta > np.pi:
            self.pose_theta -= 2.0 * np.pi
        if self.pose_theta < -np.pi:
            self.pose_theta += 2.0 * np.pi

        self.accumulated_distance += np.fabs(linear_velocity) * dt
        self.amr_data_distance_msg.data = f"{self.accumulated_distance:.3f} (m)"
        self.amr_data_distance.publish(self.amr_data_distance_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(self.pose_x)
        odom_msg.pose.pose.position.y = float(self.pose_y)

        q = quaternion_from_euler(0.0, 0.0, float(self.pose_theta))
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = float(linear_velocity)
        odom_msg.twist.twist.angular.z = float(angular_velocity)
        self.odom_pub.publish(odom_msg)

        # TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.pose_x)
        t.transform.translation.y = float(self.pose_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    # -------------------------
    # Callbacks
    # -------------------------
    def cmd_vel_callback(self, msg: Twist):
        if not self.JOY_CONTROL:
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z
            control_dt = 0.05

            velocity_right = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity)
            velocity_left = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity)

            encoder_delta_right = -(velocity_right * control_dt * float(self.md.encoder_gain)) / (np.pi * self.wheel_diameter)
            encoder_delta_left = (velocity_left * control_dt * float(self.md.encoder_gain)) / (np.pi * self.wheel_diameter)

            self.target_pos1 += encoder_delta_left
            self.target_pos2 += encoder_delta_right

    def joy_callback(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        def B(i):
            return buttons[i] if i < len(buttons) else 0

        def A(i):
            return axes[i] if i < len(axes) else 0.0

        self.joy_lr = -A(1)
        self.joy_fb = -A(2)

        self.joy_r2 = A(4)
        self.joy_l2 = A(5)
        self.joy_stop = B(0)

        self.joy_speed_up = B(11)
        self.joy_speed_down = B(10)

        if (self.joy_speed_up == 1) and (self.joy_speed_up_old == 0):
            if self.gain_count < len(self.gain_list) - 1:
                self.gain_count += 1
                self.get_logger().info(f"[GAIN+] => {self.gain_list[self.gain_count]}")

        if (self.joy_speed_down == 1) and (self.joy_speed_down_old == 0):
            if self.gain_count > 0:
                self.gain_count -= 1
                self.get_logger().info(f"[GAIN-] => {self.gain_list[self.gain_count]}")

        self.joy_speed_up_old = self.joy_speed_up
        self.joy_speed_down_old = self.joy_speed_down

        gain = self.gain_list[self.gain_count]
        self.joy_fb *= gain
        self.joy_lr *= gain * 0.5

        EPS = 1e-5
        if abs(self.joy_r2 - 1.0) < EPS and abs(self.joy_l2 - 1.0) < EPS:
            self.change_mode = 1

        if abs(self.joy_r2 + 1.0) < EPS and abs(self.joy_l2 + 1.0) < EPS and self.change_mode == 1:
            self.change_mode = 0

            with self._lock:
                pos1 = self._md_pos1
                pos2 = self._md_pos2

            self.target_pos1 = pos1
            self.target_pos2 = pos2

            self.JOY_CONTROL = not self.JOY_CONTROL
            self.get_logger().info("=== Joystick Control Mode ===" if self.JOY_CONTROL else "=== Auto Control Mode ===")


def main(args=None):
    rclpy.init(args=args)
    node = Nodelet()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
