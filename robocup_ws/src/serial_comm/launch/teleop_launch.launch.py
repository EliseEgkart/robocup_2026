from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        Node(
            package='serial_comm',
            executable='test_md_teleop_node',
            name='test_md_teleop_node',
            output='screen',
            parameters=[{'max_speed': 200}]
        ),
    ])
