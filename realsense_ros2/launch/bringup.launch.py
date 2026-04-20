from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "false",
                    "depth_module.depth_profile": "640x480x15",
                }.items(),
            )
        ]
    )
