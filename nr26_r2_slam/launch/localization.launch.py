"""
R2ロボット 自己位置推定 launch ファイル（既存地図使用）
使用機器:
  - LD19 LiDAR (ldlidar_node)
  - 3輪外付けオドメトリ (r2_odom)
  - slam_toolbox (localization_slam_toolbox_node) localizationモード

使用方法:
  ros2 launch nr26_r2_slam localization.launch.py map_file_name:=/path/to/map

引数:
  map_file_name : 保存済み地図ファイルのフルパス（拡張子なし）
                  例: /home/user/maps/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file_name = LaunchConfiguration('map_file_name', default='')

    slam_config_path = os.path.join(
        get_package_share_directory('nr26_r2_slam'),
        'config',
        'slam_toolbox_localization.yaml'
    )

    # ------------------------------------------------------------------ #
    # 1. 外付け3輪オドメトリノード（odom → base_link TF も発行）
    # ------------------------------------------------------------------ #
    r2_odom_node = Node(
        package='nr26_r2_hw_ctrl',
        executable='r2_odom',
        name='r2_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------------ #
    # 2. base_link → ldlidar_link 静的 TF
    #    LiDAR の搭載位置に合わせて x/y/z/roll/pitch/yaw を調整してください
    #    （デフォルト: ロボット中央、高さ 0.18m）
    # ------------------------------------------------------------------ #
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        output='screen',
        arguments=['0.0', '0.0', '0.18', '0.0', '0.0', '0.0',
                   'base_link', 'ldlidar_base']
    )

    # ------------------------------------------------------------------ #
    # 3. LD19 LiDAR ドライバ起動
    # ------------------------------------------------------------------ #
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar_node'),
                'launch',
                'ldlidar_bringup.launch.py'
            )
        ),
        launch_arguments={'node_name': 'ldlidar_node'}.items()
    )

    # ------------------------------------------------------------------ #
    # 4. slam_toolbox（自己位置推定モード）
    #    map_file_name 引数で読み込む地図ファイルを指定する
    # ------------------------------------------------------------------ #
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file_name}
        ]
    )

    # ------------------------------------------------------------------ #
    # 5. RViz2 可視化
    # ------------------------------------------------------------------ #
    rviz2_config = os.path.join(
        get_package_share_directory('nr26_r2_slam'),
        'config',
        'slam_mapping.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='シミュレーション時刻を使用するか'
        ),
        DeclareLaunchArgument(
            'map_file_name',
            default_value='',
            description='自己位置推定に使用する地図ファイルのフルパス（拡張子なし）'
        ),
        r2_odom_node,
        base_to_lidar_tf,
        ldlidar_launch,
        slam_toolbox_node,
        rviz2_node,
    ])
