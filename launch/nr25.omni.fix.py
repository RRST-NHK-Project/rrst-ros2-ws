from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='nr25_omni_imu',
            name='omni_fix',
            output='screen',
            emulate_tty=True
        ),
    ])
