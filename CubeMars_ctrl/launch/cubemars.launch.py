"""ブリッジ(シリアル) + ドライバ を起動する。

    ros2 launch CubeMars_ctrl cubemars.launch.py
    ros2 launch CubeMars_ctrl cubemars.launch.py serial_port:=/dev/ttyACM1

PS4操作は別ターミナルで teleop.launch.py を起動する(止めたい側だけ止められるように分けている)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('CubeMars_ctrl'), 'config', 'cubemars_ctrl.yaml')

    serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='ESP32のシリアルポート(ls /dev/ttyACM* で確認)')

    bridge = Node(
        package='CubeMars_ctrl',
        executable='cubemars_bridge_node',
        name='cubemars_bridge_node',
        output='screen',
        emulate_tty=True,
        parameters=[params, {'serial_port': LaunchConfiguration('serial_port')}],
    )

    driver = Node(
        package='CubeMars_ctrl',
        executable='cubemars_driver_node',
        name='cubemars_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )

    return LaunchDescription([serial_port, bridge, driver])
