from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    r2_md = Node(
        package='nr26_r2_hw_ctrl',
        executable='r2_md',
        name='r2_md',
        output='screen'
    )

    r2_mc = Node(
        package='nr26_r2_hw_ctrl',
        executable='r2_mc',
        name='r2_mc',
        output='screen'
    )

    r2_hand_ctrl = Node(
        package='nr26_r2_hw_ctrl',
        executable='r2_hand_ctrl',
        name='r2_hand_ctrl',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        r2_md,
        r2_mc,
        r2_hand_ctrl
    ])