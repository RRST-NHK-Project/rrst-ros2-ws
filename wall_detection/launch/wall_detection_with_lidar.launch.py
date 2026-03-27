"""LD19ライダーの立ち上げを含む壁検知ノードのlaunchファイル。

LD19が未起動の状態から壁検知を開始する場合に使用する。
LD19の起動（ldlidar_bringup.launch.py インクルード）と
wall_detection_node の起動を一括で行う。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- LD19 bringup ----
    ldlidar_node_share = get_package_share_directory('ldlidar_node')
    ldlidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_node_share, 'launch', 'ldlidar_bringup.launch.py')
        )
    )

    # ---- 壁検知パラメータ ----
    fov_half_deg_arg = DeclareLaunchArgument(
        'fov_half_deg',
        default_value='45.0',
        description='前方FOV半角 [deg]。この角度内の点群を壁検知に使用する。'
    )
    ransac_iterations_arg = DeclareLaunchArgument(
        'ransac_iterations',
        default_value='200',
        description='RANSACの試行回数'
    )
    ransac_threshold_arg = DeclareLaunchArgument(
        'ransac_threshold',
        default_value='0.05',
        description='RANSACの距離閾値 [m]'
    )
    ransac_min_inliers_arg = DeclareLaunchArgument(
        'ransac_min_inliers',
        default_value='10',
        description='インライアとみなす最低点数'
    )

    # ---- 壁検知ノード ----
    wall_detection_node = Node(
        package='wall_detection',
        executable='wall_detection_node',
        name='wall_detection_node',
        output='screen',
        parameters=[{
            'scan_topic': '/ldlidar_node/scan',
            'fov_half_deg': LaunchConfiguration('fov_half_deg'),
            'ransac_iterations': LaunchConfiguration('ransac_iterations'),
            'ransac_threshold': LaunchConfiguration('ransac_threshold'),
            'ransac_min_inliers': LaunchConfiguration('ransac_min_inliers'),
        }]
    )

    return LaunchDescription([
        fov_half_deg_arg,
        ransac_iterations_arg,
        ransac_threshold_arg,
        ransac_min_inliers_arg,
        ldlidar_bringup,
        wall_detection_node,
    ])
