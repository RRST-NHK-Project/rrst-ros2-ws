from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_camera_offset_arg = DeclareLaunchArgument(
        "use_camera_offset",
        default_value="false",
        description="Enable camera lateral offset compensation in cube alignment",
    )

    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen"
    )

    r2_md = Node(
        package="nr26_r2_hw_ctrl", executable="r2_md", name="r2_md", output="screen"
    )

    r2_mc = Node(
        package="nr26_r2_hw_ctrl", executable="r2_mc", name="r2_mc", output="screen"
    )

    r2_sc = Node(
        package="nr26_r2_hw_ctrl",
        executable="r2_sc",
        name="sequence_ctrl_node",
        output="screen",
        parameters=[
            {
                "use_camera_offset": LaunchConfiguration("use_camera_offset"),
            }
        ],
    )

    return LaunchDescription(
        [
            use_camera_offset_arg,
            joy_node,
            r2_md,
            r2_mc,
            r2_sc,
        ]
    )
