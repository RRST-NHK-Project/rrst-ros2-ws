from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Input image topic",
    )
    result_topic_arg = DeclareLaunchArgument(
        "result_topic",
        default_value="/kfs_cube_identifier/result",
        description="Output result topic",
    )
    debug_topic_arg = DeclareLaunchArgument(
        "debug_image_topic",
        default_value="/kfs_cube_identifier/debug_image",
        description="Output debug image topic",
    )
    dict_dir_arg = DeclareLaunchArgument(
        "kfs_dictionary_dir",
        default_value="/home/dev/ros2_ws/src/KFS_judgement_machines/AKAZA/KFS Image V1.0",
        description="Directory containing KFS dictionary images",
    )
    use_realsense_arg = DeclareLaunchArgument(
        "use_realsense",
        default_value="true",
        description="Start realsense_ros2 bringup.launch.py",
    )
    show_debug_viewer_arg = DeclareLaunchArgument(
        "show_debug_viewer",
        default_value="true",
        description="Show OpenCV window for debug image",
    )

    realsense_process = ExecuteProcess(
        cmd=["ros2", "launch", "realsense_ros2", "bringup.launch.py"],
        condition=IfCondition(LaunchConfiguration("use_realsense")),
        output="screen",
    )

    node = Node(
        package="kfs_cube_identifier",
        executable="kfs_cube_identifier_node",
        name="kfs_cube_identifier",
        output="screen",
        additional_env={"PYTHONNOUSERSITE": "1"},
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "result_topic": LaunchConfiguration("result_topic"),
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                "kfs_dictionary_dir": LaunchConfiguration("kfs_dictionary_dir"),
            }
        ],
    )

    debug_viewer_node = Node(
        package="kfs_cube_identifier",
        executable="kfs_debug_image_viewer",
        name="kfs_debug_image_viewer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_debug_viewer")),
        additional_env={"PYTHONNOUSERSITE": "1"},
        parameters=[
            {
                "image_topic": LaunchConfiguration("debug_image_topic"),
                "window_name": "KFS Debug Image",
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            result_topic_arg,
            debug_topic_arg,
            dict_dir_arg,
            use_realsense_arg,
            show_debug_viewer_arg,
            realsense_process,
            node,
            debug_viewer_node,
        ]
    )
