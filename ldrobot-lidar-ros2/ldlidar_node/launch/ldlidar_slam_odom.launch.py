# ldlidar_slam_odom.launch.py（修正版）
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Lifecycle manager configurationファイル
    lc_mgr_config_path = os.path.join(
        get_package_share_directory("ldlidar_node"), "params", "lifecycle_mgr_slam.yaml"
    )

    # SLAM Toolbox configurationファイル
    slam_config_path = os.path.join(
        get_package_share_directory("ldlidar_node"), "params", "slam_toolbox.yaml"
    )

    # Lifecycle managerノード
    lc_mgr_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[lc_mgr_config_path],
    )

    # SLAM Toolboxノード（LifecycleNode, autostartで自動アクティブ化）
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace="",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config_path],
        remappings=[("/scan", "/ldlidar_node/scan")],
        autostart=True,  # ←重要
    )

    # LDLidar起動（launchファイル読み込み）
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("ldlidar_node"),
                "/launch/ldlidar_bringup.launch.py",
            ]
        ),
        launch_arguments={"node_name": "ldlidar_node"}.items(),
    )

    # RViz2ノード
    rviz2_config = os.path.join(
        get_package_share_directory("ldlidar_node"), "config", "ldlidar_slam.rviz"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[["-d"], [rviz2_config]],
    )

    # LaunchDescriptionに追加
    ld = LaunchDescription()
    ld.add_action(ldlidar_launch)
    ld.add_action(lc_mgr_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz2_node)

    return ld
