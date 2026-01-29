#!/usr/bin/env python3

#=====================================================#
# 기능: ROS2 TTS Speaker
# - 포트를 설정하고 출력 장치에 저장해놓은 음성 및 사운드 데이터 출력
# - 다른 노드로 부터 String 트리거를 받아서 해당 함수에서 검증 및 출력
#
# TODO : speaker class 추가 구성
# 최종 수정일: 2025.11.09
# 편집자: 김형진
#=====================================================#
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pygame import mixer


class Speaker(Node):
    def __init__(self):
        super().__init__('speaker_node')
        mixer.init()

        # 런치 파일에서 파라미터 받아오기
        self.declare_parameter('sound_path', '')
        self.sound_path = self.get_parameter('sound_path').get_parameter_value().string_value
        if not self.sound_path:
            self.sound_path = os.path.join(os.getcwd(), 'sounds')

        self.get_logger().info(f"[Speaker] Using sound path: {self.sound_path}")

        self.sub_speaker = self.create_subscription(String, '/speaker', self.play_sound_callback, 5)

    def play_sound_callback(self, msg: String):
        data = msg.data
        self.get_logger().info(f"Received: {data}")

        if data == "test_effect":
            self.play_effect('alarm_test.mp3', -1)
            # 작업완료 시 아래 로그는 지운다.
            self.get_logger().info("Data Recived test effect trigger")


        elif data == "test_voice":
            self.play_voice('voice_test.mp3', 1)
            # 작업완료 시 아래 로그는 지운다.
            self.get_logger().info("Data Recived test voice trigger ")

        else:
            self.get_logger().warn(f"Undefined message: {data}")

    def play_effect(self, name, repeat):
        mixer.music.load(os.path.join(self.sound_path, name))
        mixer.music.play(repeat)

    def play_voice(self, name, repeat):
        mixer.music.load(os.path.join(self.sound_path, name))
        mixer.music.play(repeat)
        while mixer.music.get_busy():
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = Speaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Speaker Node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


