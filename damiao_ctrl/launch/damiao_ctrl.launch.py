from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_joy = LaunchConfiguration('use_joy')
    serial_port = LaunchConfiguration('serial_port')
    config_file = PathJoinSubstitution(
        [FindPackageShare('damiao_ctrl'), 'config', 'damiao_ctrl.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy', default_value='true',
            description='PS4コントローラのテレオペ(joy_node + ps4_teleop_node)を起動するか'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='ESP32のUSBシリアルポート'),

        Node(
            package='damiao_ctrl',
            executable='bridge_node',
            name='damiao_bridge_node',
            output='screen',
            parameters=[config_file, {'serial_port': serial_port}],
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
