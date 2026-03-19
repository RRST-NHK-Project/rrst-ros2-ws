from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ArUco検出ノードとビューアノードを同時に起動するlaunchファイル."""
    return LaunchDescription([
        Node(
            package='aruco_tracker',
            executable='aruco_test',
            name='aruco_pose_publisher',
            output='screen',
        ),
        Node(
            package='aruco_tracker',
            executable='aruco_viewer',
            name='aruco_viewer',
            output='screen',
        ),
    ])
