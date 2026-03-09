from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    r1_sd = Node(
        package='nr26_r1_hw_ctrl',
        executable='r1_sd',
        name='r1_sd',
        output='screen'
    )

    r1_mc = Node(
        package='nr26_r1_hw_ctrl',
        executable='r1_mc',
        name='r1_mc',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        r1_sd,
        r1_mc
    ])