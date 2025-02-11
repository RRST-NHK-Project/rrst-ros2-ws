from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[{"device_id": 1}],  # ここで device_id を指定
            remappings=[("/joy", "/joy1")]
        ),
        Node(
            package='f7_udp_cpp',
            executable='nr25_dr_sd',
            output='screen',
        ),
        Node(
            package='f7_udp_cpp',
            executable='nr25_dr',
            output='screen',
        ),
    ])
