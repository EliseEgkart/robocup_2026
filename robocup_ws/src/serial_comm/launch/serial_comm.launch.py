from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # 1) 실제 로봇 하드웨어 구동 노드 (모터 + odom + joint_states)
    base_main_node = Node(
        package="serial_comm",
        executable="base_main_node",
        name="base_main_node",
        output="screen"
    )
    ld.add_action(base_main_node)

    # 2) 조이스틱 노드
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )
    ld.add_action(joy_node)


    # 3) URDF 로딩 후 robot_state_publisher
    pkg_path = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_path, 'description', 'robot_description.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    ld.add_action(node_robot_state_publisher)

    return ld
