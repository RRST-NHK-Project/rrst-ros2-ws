from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2can',
            executable='ros2can',
            name='ros2can_gui',
            output='screen',
        ),
    ])
