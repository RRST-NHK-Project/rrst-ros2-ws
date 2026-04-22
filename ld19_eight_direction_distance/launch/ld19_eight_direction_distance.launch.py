from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/ldlidar_node/scan",
        description="LaserScan topic from LD19 driver",
    )
    publish_topic_arg = DeclareLaunchArgument(
        "publish_topic",
        default_value="/ld19/eight_direction_distance",
        description="Published topic containing 8-direction distances (Float32MultiArray)",
    )
    output_hz_arg = DeclareLaunchArgument(
        "output_hz",
        default_value="2.0",
        description="Log output frequency",
    )
    window_deg_arg = DeclareLaunchArgument(
        "window_deg",
        default_value="2.0",
        description="Half window for angle match in degrees",
    )
    front_angle_deg_arg = DeclareLaunchArgument(
        "front_angle_deg",
        default_value="0.0",
        description="Offset angle where robot front is defined",
    )

    node = Node(
        package="ld19_eight_direction_distance",
        executable="ld19_eight_direction_node",
        name="ld19_eight_direction_node",
        output="screen",
        parameters=[
            {
                "scan_topic": LaunchConfiguration("scan_topic"),
                "publish_topic": LaunchConfiguration("publish_topic"),
                "output_hz": LaunchConfiguration("output_hz"),
                "window_deg": LaunchConfiguration("window_deg"),
                "front_angle_deg": LaunchConfiguration("front_angle_deg"),
            }
        ],
    )

    return LaunchDescription(
        [
            scan_topic_arg,
            publish_topic_arg,
            output_hz_arg,
            window_deg_arg,
            front_angle_deg_arg,
            node,
        ]
    )
