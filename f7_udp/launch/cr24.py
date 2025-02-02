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
            executable='setoshio',
            name='main',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_main',
            name='main',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_gui',
            name='gui',
            emulate_tty=True
        ),
        Node(
            package='f7_udp',
            executable='cr24_main_manual2',
            name='manual',
            emulate_tty=True
        ),
    ])
