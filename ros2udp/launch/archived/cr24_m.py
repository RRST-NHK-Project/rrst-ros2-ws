from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_main_manual',
            name='main',
            output='screen',
            emulate_tty=True
        ),
    ])
