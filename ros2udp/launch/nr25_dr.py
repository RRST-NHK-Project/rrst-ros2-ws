from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                output="screen",
                parameters=[{"device_id": 0}],  # ここで device_id を指定
                remappings=[("/joy", "/joy1")],
            ),
            Node(
                package="ros2udp",
                executable="nr25_dr_sd",
                output="screen",
            ),
            Node(
                package="ros2udp",
                executable="nr25_dr_c610",
                output="screen",
            ),
        ]
    )
