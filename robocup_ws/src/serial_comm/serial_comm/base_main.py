import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header, Bool
from sensor_msgs.msg import Joy
from amr_msgs.msg import WheelMotor
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2

from serial_comm.motor_driver import MotorDriver


class Nodelet(Node):
    def __init__(self):
        super().__init__('base_main')
        self.pub = self.create_publisher(WheelMotor, '/wheelmotor', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 100)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 100)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.amr_data_distance = self.create_publisher(String, '/amr_data_distance', 10)

        self.dt = 0.02
        self.timer_ = self.create_timer(self.dt, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

        ####6/19 global coordinate####
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.loopcnt = 0

        self.firstloop = True
        self.JOY_CONTROL = False

        # Motor driver class
        self.md = MotorDriver()

        # PID related variables
        self.p_gain = 1.
        self.i_gain = 0.
        self.d_gain = 0.01
        self.forget = 0.99

        self.err1_prev, self.err1_i = 0., 0.
        self.err2_prev, self.err2_i = 0., 0.
        self.torque1, self.torque2 = 0, 0
        self.velocity1, self.velocity2 = 0, 0

        # target position
        self.target_pos1, self.target_pos2 = 0, 0

        # joy gain
        self.joy_fb = 0
        self.joy_lr = 0
        self.v_gain = 100
        self.w_gain = 50

        self.joy_r2 = 0
        self.joy_l2 = 0
        self.change_mode = 0
        self.joy_stop = 0

        self.joy_speed_up = 0
        self.joy_speed_down = 0
        self.gain_count = 2
        self.gain_list = [0.5, 0.7, 1.0, 1.3, 1.5, 1.7, 2.0, 2.3, 2.5, 4.0]

        # >>> 수정: 버튼 edge 검출용 이전 값 초기화
        self.joy_speed_up_old = 0
        self.joy_speed_down_old = 0
        # <<< 수정 끝

        self.msg_wheelmotor = WheelMotor()

        # odom param
        self.wheel_separation = 0.32  # Adjust as necessary
        self.wheel_diameter = 0.13    # Adjust as necessary
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.last_time = self.get_clock().now()
        # odom -> base_link yaw offset (z축 기준 180도 회전)
        self.yaw_offset = np.pi

        # accumulated_distance
        self.accumulated_distance = 0.0

        # Encoder positions (엔코더 누적/증분용)
        self.last_pos1 = 0.0
        self.last_pos2 = 0.0
        self.cur_pos1 = 0.0
        self.cur_pos2 = 0.0
        self.del_pos1 = 0.0
        self.del_pos2 = 0.0

        self.vel_input1 = 0.0
        self.vel_input2 = 0.0
        self.vel_input1_old = 0.0
        self.vel_input2_old = 0.0

        # lowpass filter
        self.v_motor_last = 0.0
        self.w_motor_last = 0.0
        self.alpha = 0.5

        self.target_marker_id = 3

        # aruco marker flag
        self.marker_detected = False
        self.marker_detected_count = 0
        self.marker_deadreckonmode = False
        self.marker_target_distance = 0.9
        self.marker_target_distance2 = 0.9
        self.marker_target_encoder = self.marker_target_distance / (np.pi * self.wheel_diameter / self.md.encoder_gain)
        self.marker_moving_distance = 0
        self.marker_moving_distance2 = 0
        self.next_marker_id = 7  # Update target marker ID to the next marker
        self.marker_rotation_en_count = 0.
        self.marker7_1 = True

        # -------------------------
        # DEBUG: motor rx logging / timer lag
        # -------------------------
        self.DEBUG_MOTOR_RX = True          # 모터 수신(raw) 로그 ON/OFF
        self.DEBUG_MOTOR_RX_HZ = 10.0       # 초당 몇 번 찍을지 (5~20 권장)
        self._last_motor_log_time = self.get_clock().now()

        self.DEBUG_TIMER_LAG = True         # timer callback 실제 주기 확인
        self._last_timer_time = self.get_clock().now()

    def timer_callback(self):
        self.loopcnt += 1

        # -------------------------
        # DEBUG: timer callback 주기(밀림) 체크
        # -------------------------
        if self.DEBUG_TIMER_LAG:
            now_t = self.get_clock().now()
            dt_timer = (now_t - self._last_timer_time).nanoseconds / 1e9
            self._last_timer_time = now_t
            # 기대(0.02s)보다 많이 밀리면 경고
            if dt_timer > 0.05:
                self.get_logger().warn(f"[TIMER_LAG] dt_timer={dt_timer:.3f}s (expected ~{self.dt:.3f}s)")

        if self.firstloop:
            self.md.send_vel_cmd(self.velocity1, self.velocity2)
            self.md.recv_motor_state()
            self.target_pos1 = self.md.pos1
            self.target_pos2 = self.md.pos2

            # 오도메트리/JointState 기준 엔코더 0점
            self.del_pos1 = self.md.pos1
            self.del_pos2 = self.md.pos2

            self.firstloop = False
            return

        if self.JOY_CONTROL:
            self.vel_input1 = self.v_gain * self.joy_fb
            self.vel_input1 -= self.w_gain * self.joy_lr
            self.vel_input2 = self.v_gain * self.joy_fb
            self.vel_input2 += self.w_gain * self.joy_lr

            self.vel_input1 = self.Lowpass_filter(self.vel_input1, self.vel_input1_old, 0.1)
            self.vel_input2 = self.Lowpass_filter(self.vel_input2, self.vel_input2_old, 0.1)

            self.vel_input1_old = self.vel_input1
            self.vel_input2_old = self.vel_input2

            if self.joy_stop == 1:
                self.md.send_position_cmd(self.md.pos1, self.md.pos2, int(60), int(60))
                self.get_logger().info('stop')
            else:
                self.md.send_vel_cmd(self.vel_input1, self.vel_input2)

            self.msg_wheelmotor.target1 = int(self.vel_input1)
            self.msg_wheelmotor.target2 = int(self.vel_input2)

        else:
            if self.marker_detected and not self.marker_deadreckonmode:
                self.marker_detected_count += 1
                self.get_logger().info(f'count: {self.marker_detected_count}')
            if self.marker_detected_count > 20:
                self.marker_deadreckonmode = True
                self.marker_moving_distance = 0
                self.marker_detected_count = 0

            if self.marker_deadreckonmode and self.target_marker_id == 3:
                vel_meter = self.marker_target_distance / 4.
                vel_enc = vel_meter / (np.pi * self.wheel_diameter / self.md.encoder_gain)
                if self.marker_moving_distance < self.marker_target_distance:
                    self.target_pos1 += vel_enc * self.dt
                    self.target_pos2 += vel_enc * self.dt
                    self.marker_moving_distance += vel_meter * self.dt
                else:
                    self.marker_deadreckonmode = False
                    self.target_marker_id = self.next_marker_id
                    self.marker_moving_distance = 0.
            if self.marker_deadreckonmode and self.target_marker_id == 7:
                wheel_separation = 0.298
                wheel_diameter = 0.17
                encoder_pulses_per_wheel = 240
                vel_meter = self.marker_target_distance2 / 4.
                vel_enc = vel_meter / (np.pi * self.wheel_diameter / self.md.encoder_gain)
                R = wheel_separation / 2
                distance_per_wheel = (np.pi * R) / 2
                wheel_circumference = np.pi * wheel_diameter
                rotation_encoder_count = (distance_per_wheel / wheel_circumference) * encoder_pulses_per_wheel

                if self.marker_rotation_en_count < rotation_encoder_count and self.marker7_1 is True:
                    self.target_pos1 += rotation_encoder_count / 100
                    self.target_pos2 -= rotation_encoder_count / 100
                    self.marker_rotation_en_count += rotation_encoder_count / 100
                elif self.marker_moving_distance2 < self.marker_target_distance2 and self.marker7_1 is False:
                    self.target_pos1 += vel_enc * self.dt
                    self.target_pos2 += vel_enc * self.dt
                    self.marker_moving_distance2 += vel_meter * self.dt
                else:
                    if (self.marker_rotation_en_count > rotation_encoder_count) and (self.marker_moving_distance2 > self.marker_target_distance2):
                        self.marker_deadreckonmode = False
                        self.marker_moving_distance2 = 0.
                        self.marker_rotation_en_count = 0.
                        self.target_marker_id = 3
                    self.marker7_1 is False

            self.marker_detected = False

            self.md.send_position_cmd(int(self.target_pos1), int(self.target_pos2), int(60), int(60))

            self.msg_wheelmotor.target1 = int(self.target_pos1)
            self.msg_wheelmotor.target2 = int(self.target_pos2)

        # -------------------------
        # 모터 상태 수신
        # -------------------------
        self.md.recv_motor_state()

        # -------------------------
        # DEBUG: recv 직후 raw 값 로깅 (throttle)
        # - 콘솔 로그가 과하면 오히려 루프가 밀리니 주기 제한
        # -------------------------
        if self.DEBUG_MOTOR_RX:
            now = self.get_clock().now()
            dt_log = (now - self._last_motor_log_time).nanoseconds / 1e9
            if dt_log >= (1.0 / self.DEBUG_MOTOR_RX_HZ):
                self._last_motor_log_time = now

                pos1 = self.md.pos1
                pos2 = self.md.pos2
                rpm1 = self.md.rpm1
                rpm2 = self.md.rpm2
                cur1 = self.md.current1
                cur2 = self.md.current2

                # joint_states에서 쓰는 "상대 엔코더"도 같이 보기
                rel1 = pos1 - self.del_pos1
                rel2 = pos2 - self.del_pos2

                self.get_logger().info(
                    f"[MOTOR_RX] pos1={pos1} pos2={pos2} | "
                    f"rpm1={rpm1:.2f} rpm2={rpm2:.2f} | "
                    f"cur1={int(cur1)} cur2={int(cur2)} | "
                    f"rel1={rel1:.1f} rel2={rel2:.1f}"
                )

        # WheelMotor 메시지 채우기
        self.msg_wheelmotor.position1 = self.md.pos1
        self.msg_wheelmotor.position2 = self.md.pos2
        self.msg_wheelmotor.velocity1 = self.md.rpm1
        self.msg_wheelmotor.velocity2 = self.md.rpm2
        self.msg_wheelmotor.current1 = int(self.md.current1)
        self.msg_wheelmotor.current2 = int(self.md.current2)

        # >>> 수정: 모터 드라이버에서 이미 좌측 부호 보정됨 (추가 반전 제거)
        rpm_left = self.md.rpm1     # left wheel
        rpm_right = self.md.rpm2    # right wheel

        self.msg_wheelmotor.v_x = (rpm_left + rpm_right) * np.pi * self.wheel_diameter / (60 * 2)
        self.msg_wheelmotor.w_z = -(rpm_right - rpm_left) * np.pi * self.wheel_diameter / (60 * self.wheel_separation)
        # <<< 수정 끝

        self.pub.publish(self.msg_wheelmotor)

        ############################# odom 5/30 ################################

        # 기준점(del_pos1/2) 기준 누적 엔코더 값
        self.cur_pos1 = self.md.pos1 - self.del_pos1
        self.cur_pos2 = self.md.pos2 - self.del_pos2

        # >>> JointState: 엔코더 누적값 → 바퀴 각도(rad)로 변환 후 퍼블리시
        # URDF 축 방향 보정: 좌측만 반전 (좌측 TF 회전 방향 보정)
        left_enc_rel = self.cur_pos1      # left wheel (sign flipped for TF)
        right_enc_rel = -self.cur_pos2      # right wheel

        left_pos_rad = 2.0 * np.pi * (left_enc_rel / self.md.encoder_gain)
        right_pos_rad = 2.0 * np.pi * (right_enc_rel / self.md.encoder_gain)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_left_joint', 'wheel_right_joint']   # URDF 조인트 이름과 맞춰야 함
        js.position = [left_pos_rad, right_pos_rad]
        self.joint_state_pub.publish(js)
        # <<< JointState 퍼블리시 끝

        # 증분 엔코더 값 (오돔 계산용)
        delta_pos1 = self.cur_pos1 - self.last_pos1
        delta_pos2 = self.cur_pos2 - self.last_pos2

        # Update last encoder positions
        self.last_pos1 = self.cur_pos1
        self.last_pos2 = self.cur_pos2

        # >>> 수정: 오돔용 엔코더 부호 보정 제거 (드라이버에서 이미 보정)
        delta_left_enc = delta_pos1       # left wheel encoder
        delta_right_enc = delta_pos2      # right wheel encoder

        left_wheel_disp = (delta_left_enc / self.md.encoder_gain) * (np.pi * self.wheel_diameter)
        right_wheel_disp = (delta_right_enc / self.md.encoder_gain) * (np.pi * self.wheel_diameter)
        # <<< 수정 끝

        # 시간 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 선속도 / 각속도
        # 드라이버에서 좌측 부호 보정이 끝난 상태이므로 추가 반전 제거
        linear_velocity = (left_wheel_disp + right_wheel_disp) / (2.0 * dt)
        angular_velocity = (right_wheel_disp - left_wheel_disp) / (self.wheel_separation * dt)

        # 포즈 적분
        self.pose_x += linear_velocity * np.cos(self.pose_theta) * dt
        self.pose_y += linear_velocity * np.sin(self.pose_theta) * dt
        self.pose_theta += angular_velocity * dt
        if self.pose_theta > np.pi:
            self.pose_theta = self.pose_theta - (2 * np.pi)
        if self.pose_theta < -np.pi:
            self.pose_theta = self.pose_theta + (2 * np.pi)

        # 누적 거리
        self.accumulated_distance += np.fabs(linear_velocity) * dt
        amr_data_distance_ = String()
        amr_msg_string = f"{self.accumulated_distance:.3f} (m)"
        amr_data_distance_.data = amr_msg_string
        self.amr_data_distance.publish(amr_data_distance_)

        # 오돔 메시지 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y

        q = quaternion_from_euler(0, 0, self.pose_theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

        # TF (odom → base_link)
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = 0.
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

    def transform_pose_to_map(self, x, y, theta, transform):
        tx = transform.translation.x
        ty = transform.translation.y

        q = transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        transformation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), tx],
            [np.sin(yaw), np.cos(yaw), ty],
            [0, 0, 1]
        ])

        pose_odom = np.array([x, y, 1])
        pose_map = np.dot(transformation_matrix, pose_odom)

        theta_map = theta + yaw
        if theta_map > np.pi:
            theta_map = theta_map - (2 * np.pi)
        if theta_map < -np.pi:
            theta_map = theta_map + (2 * np.pi)

        return pose_map[0], pose_map[1], theta_map

    def cmd_vel_callback(self, msg):
        if not self.JOY_CONTROL:
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z
            control_dt = 0.05

            velocity_right = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity)
            velocity_left = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity)

            encoder_delta_right = (velocity_right * control_dt * self.md.encoder_gain) / (np.pi * self.wheel_diameter)
            encoder_delta_left = (velocity_left * control_dt * self.md.encoder_gain) / (np.pi * self.wheel_diameter)

            self.target_pos1 += encoder_delta_left
            self.target_pos2 += encoder_delta_right

    def position_control(self, target1, target2):
        err1 = target1 - self.md.pos1
        self.err1_i = self.forget * (self.err1_i + err1 * self.dt)
        err1_d = (err1 - self.err1_prev) / self.dt
        self.err1_prev = err1

        self.velocity1 = self.p_gain * err1
        self.velocity1 += self.i_gain * self.err1_i
        self.velocity1 += self.d_gain * err1_d

        if self.velocity1 > 1022:
            self.velocity1 = 1022
        elif self.velocity1 < -1022:
            self.velocity1 = -1022
        self.velocity1 = np.array(self.velocity1, dtype=np.int16)

        err2 = target2 - self.md.pos2
        self.err2_i = self.forget * (self.err2_i + err2 * self.dt)
        err2_d = (err2 - self.err2_prev) / self.dt
        self.err2_prev = err2

        self.velocity2 = self.p_gain * err2
        self.velocity2 += self.i_gain * self.err2_i
        self.velocity2 += self.d_gain * err2_d

        if self.velocity2 > 1022:
            self.velocity2 = 1022
        elif self.velocity2 < -1022:
            self.velocity2 = -1022
        self.velocity2 = np.array(self.velocity2, dtype=np.int16)

    def send_data(self, data):
        self.get_logger().info(f'send data check: {data}')
        self.serial_port.write(data.encode())

    def joy_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes

        def B(i):
            return buttons[i] if i < len(buttons) else 0

        def A(i):
            return axes[i] if i < len(axes) else 0.0

        self.joy_fb = A(1)
        self.joy_lr = A(2)
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
            self.target_pos1 = self.md.pos1
            self.target_pos2 = self.md.pos2
            self.vel_input1 = 0.0
            self.vel_input2 = 0.0

            self.JOY_CONTROL = not self.JOY_CONTROL
            if self.JOY_CONTROL:
                self.get_logger().info("=== Joystick Control Mode ===")
            else:
                self.get_logger().info("=== Auto Control Mode ===")

    def Lowpass_filter(self, vel_input, vel_input_1, alpha):
        return alpha * vel_input + (1 - alpha) * vel_input_1


def main(args=None):
    rclpy.init(args=args)
    node = Nodelet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
