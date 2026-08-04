import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('esc_ctrl'), 'config', 'esc_ctrl.yaml')

    return LaunchDescription([
        Node(
            package='esc_ctrl',
            executable='esc_ctrl_node',
            name='esc_ctrl_node',
            output='screen',
            parameters=[config],
        ),
    ])
