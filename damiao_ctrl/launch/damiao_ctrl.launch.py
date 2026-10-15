from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# PC<->ESP32の通信は ros2can を別ターミナルで起動して使う:
#   ros2 run ros2can ros2can --nogui
# (ダッシュボード表示があるためlaunchには含めない)。use_builtin_bridge:=true で
# 旧来のbridge_nodeを代わりに起動できる(ros2canと同時に使わないこと)。


def generate_launch_description():
    use_joy = LaunchConfiguration('use_joy')
    use_builtin_bridge = LaunchConfiguration('use_builtin_bridge')
    serial_port = LaunchConfiguration('serial_port')
    config_file = PathJoinSubstitution(
        [FindPackageShare('damiao_ctrl'), 'config', 'damiao_ctrl.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy', default_value='false',
            description='PS4コントローラのテレオペ(joy_node + ps4_teleop_node)を起動するか '
                        '(PS4の実機確認が済むまで既定はfalse)'),
        DeclareLaunchArgument(
            'use_builtin_bridge', default_value='false',
            description='ros2canの代わりに旧来のbridge_nodeを起動するか'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='ESP32のUSBシリアルポート(use_builtin_bridge:=trueの時のみ使用)'),

        Node(
            package='damiao_ctrl',
            executable='bridge_node',
            name='damiao_bridge_node',
            output='screen',
            parameters=[config_file, {'serial_port': serial_port}],
            condition=IfCondition(use_builtin_bridge),
        ),
        Node(
            package='damiao_ctrl',
            executable='damiao_driver_node',
            name='damiao_driver_node',
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
            package='damiao_ctrl',
            executable='ps4_teleop_node',
            name='ps4_teleop_node',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(use_joy),
        ),
    ])
