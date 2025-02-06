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
            package='f7_udp_cpp',
            executable='nr25_dr_sd',
            name='nr25_dr_sd',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='f7_udp_cpp',
            executable='nr25_dr',
            name='dr_sd',
            output='screen',
            emulate_tty=True
        ),
    ])
