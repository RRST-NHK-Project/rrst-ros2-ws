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
            executable='cr24_exh_yolo',
            name='main',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_exh_gui',
            name='main',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_exh_automatic',
            name='gui',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_exh_manual',
            name='manual',
            output='screen',
            emulate_tty=True
        ),
    ])
