from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="kfs_akaze_detection",
                executable="kfs_akaze_detector",
                name="kfs_akaze_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/camera/color/image_raw",
                        "template_package": "kfs_pkg",
                        "template_image_name": "KFS_image_list.png",
                        "query_scale": 0.5,
                        "match_ratio": 0.87,
                        "minimum_good_matches": 14,
                        "depth_info_topic": "/kfs_akaze_detection/depth_observation",
                        "enable_depth_adapter": True,
                        "output_topic": "/kfs_akaze_detection/result",
                        "detected_topic": "/kfs_akaze_detection/detected",
                        "debug_image_topic": "/kfs_akaze_detection/debug_image",
                    }
                ],
            )
        ]
    )
