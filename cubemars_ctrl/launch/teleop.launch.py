"""PS4コントローラ(joy_node) + teleop を起動する。cubemars.launch.py とは別ターミナルで使う。

    ros2 launch cubemars_ctrl teleop.launch.py

操作: R1を押している間だけ有効 / □でモード切替 / 速度モード: 左スティック上下 /
      角度モード: △+90deg ×-90deg ○0deg 左スティック上下で微調整
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cubemars_ctrl'), 'config', 'cubemars_ctrl.yaml')

    joy_dev = DeclareLaunchArgument(
        'joy_device_id', default_value='0',
        description='joy_nodeのデバイス番号(ros2 run joy joy_enumerate_devices で確認)')

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device_id'),
            'deadzone': 0.05,
            # 入力が変化しなくても20Hzで/joyを出し続ける(teleop側の途絶検出に必要)
            'autorepeat_rate': 20.0,
        }],
    )

    teleop = Node(
        package='cubemars_ctrl',
        executable='ps4_teleop_node',
        name='ps4_teleop_node',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )

    return LaunchDescription([joy_dev, joy, teleop])
