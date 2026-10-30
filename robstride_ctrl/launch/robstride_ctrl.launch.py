from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# PC<->ESP32の通信は ros2can を別ターミナルで起動して使う:
#   ros2 run ros2can ros2can --nogui
# (ダッシュボード表示があるためlaunchには含めない)。


def generate_launch_description():
    use_joy = LaunchConfiguration('use_joy')
    config_file = PathJoinSubstitution(
        [FindPackageShare('robstride_ctrl'), 'config', 'robstride_ctrl.yaml'])

    return LaunchDescription([
        # このlaunchで起動するノードは同じPC内だけで通信する。既定(SUBNET)だと
        # 同じWi-Fiの他のPCのノードとつながってしまい、2026-10-15に他PCのjoy_nodeの
        # /joyが混ざってR1が押す/離すを繰り返した(damiao_ctrl)。
        # ros2canも同じPCで ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST を付けて起動すること。
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        DeclareLaunchArgument(
            'use_joy', default_value='false',
            description='PS4コントローラのテレオペ(joy_node + ps4_teleop_node)を起動するか'),

        Node(
            package='robstride_ctrl',
            executable='robstride_driver_node',
            name='robstride_driver_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # autorepeat_rate: 入力に変化が無くても/joyを送り直す周期[Hz]。
            # ps4_teleop_nodeはこれを生存確認に使う(joy_timeout_sec)ので0にしないこと。
            parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
            condition=IfCondition(use_joy),
        ),
        Node(
            package='robstride_ctrl',
            executable='ps4_teleop_node',
            name='ps4_teleop_node',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(use_joy),
        ),
    ])
