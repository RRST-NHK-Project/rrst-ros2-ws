from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kfs_cube_fusion",
                executable="kfs_cube_fusion_node",
                name="kfs_cube_fusion_node",
                output="screen",
                parameters=[
                    {
                        "color_topic": "/camera/camera/color/image_raw",
                        "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                        "sync_slop_sec": 0.08,
                        "template_package": "kfs_pkg",
                        "template_image_name": "KFS_image_list.png",
                        "query_scale": 0.5,
                        "match_ratio": 0.87,
                        "minimum_good_matches": 12,
                        "max_matches_to_use": 80,
                        "central_window_px": 40,
                        "depth_band_mm": 180.0,
                        "min_area_px": 350,
                        "max_center_offset_px": 180,
                        "morph_kernel_px": 5,
                        "depth_sample_window_px": 7,
                        "output_topic": "/kfs_cube_fusion/result",
                        "detected_topic": "/kfs_cube_fusion/detected",
                        "debug_image_topic": "/kfs_cube_fusion/debug_image",
                    }
                ],
            ),
        ]
    )
