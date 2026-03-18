import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():

    node_name = LaunchConfiguration("node_name")

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory("ldlidar_node"), "params", "lifecycle_mgr_slam.yaml"
    )

    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(
        get_package_share_directory("ldlidar_node"), "params", "slam_toolbox.yaml"
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[lc_mgr_config_path],
    )

    # SLAM Toolbox node in async mode
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace="",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config_path,
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
        ],
        remappings=[("/scan", "/ldlidar_node/scan")],
    )

    # --- 追加: 実際のオドメトリ発行ノード (C++コード側) ---
    # ※package名とexecutable名は、ご自身のCMakeLists.txtの設定に合わせて書き換えてください
    real_odom_node = Node(
        package="nr26_r2_hw_ctrl",
        executable="r2_odom",
        name="r2_odom",
        output="log",
    )

    # --- 追加: base_link から LIDAR への静的TF ---
    # オドメトリが odom -> base_link を作るので、
    # センサーの位置(base_linkから見たLIDARの位置)を定義する必要があります。
    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_broadcaster",
        output="screen",
        # 引数: x y z yaw pitch roll parent_frame child_frame
        # ここではLIDARが機体中心(0,0,0)にあると仮定しています
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "ldlidar_link"],
    )

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("ldlidar_node"),
                "/launch/ldlidar_bringup.launch.py",
            ]
        ),
        launch_arguments={"node_name": "ldlidar_node"}.items(),
    )

    # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory("ldlidar_node"), "config", "ldlidar_slam.rviz"
    )

    # RVIZ2 node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[["-d"], [rviz2_config]],
    )

    ld = LaunchDescription()

    # 1. ライフサイクルマネージャー
    ld.add_action(lc_mgr_node)
    # 2. SLAM Toolbox
    ld.add_action(slam_toolbox_node)
    # 3. 実際のオドメトリノード (C++)
    ld.add_action(real_odom_node)
    # 4. ロボットとLIDARの相対位置関係のTF
    ld.add_action(base_to_lidar_tf)
    # 5. LIDARドライバ
    ld.add_action(ldlidar_launch)
    # 6. 可視化
    ld.add_action(rviz2_node)

    return ld
