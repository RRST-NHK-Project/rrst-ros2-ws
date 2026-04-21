from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    r2_auto = Node(
        package="nr26_r2_hw_ctrl",
        executable="r2_auto",
        name="r2_auto",
        output="screen",
    )

    r2_sc = Node(
        package="nr26_r2_hw_ctrl",
        executable="r2_sc",
        name="sequence_ctrl_node",
        output="screen",
    )

    r2_mc = Node(
        package="nr26_r2_hw_ctrl",
        executable="r2_mc",
        name="r2_mc",
        output="screen",
    )

    return LaunchDescription(
        [
            r2_sc,
            r2_mc,
            r2_auto,
        ]
    )
