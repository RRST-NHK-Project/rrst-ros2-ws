from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    viewer = LaunchConfiguration("viewer").perform(context)

    nodes = [
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
        )
    ]

    if viewer == "builtin":
        nodes.append(
            Node(
                package="kfs_cube_fusion",
                executable="kfs_cube_fusion_viewer",
                name="kfs_cube_fusion_viewer",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/kfs_cube_fusion/debug_image",
                        "window_name": "kfs_cube_fusion debug viewer",
                    }
                ],
            )
        )
    elif viewer == "rqt_image_view":
        nodes.append(
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="kfs_cube_fusion_viewer",
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "viewer",
                default_value="none",
                description="Select viewer to start with the fusion node: none, builtin, or rqt_image_view",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
