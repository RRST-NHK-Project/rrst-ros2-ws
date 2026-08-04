import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('serial_rx_unwrap'), 'config', 'serial_rx_unwrap.yaml')

    return LaunchDescription([
        Node(
            package='serial_rx_unwrap',
            executable='serial_rx_unwrap_node',
            name='serial_rx_unwrap_node',
            output='screen',
            parameters=[config],
        ),
    ])
