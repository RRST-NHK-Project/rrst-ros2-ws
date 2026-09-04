"""NHK学生ロボコン2027(ABU Robocon 2027)フィールドをGazebo(gz-sim)で起動し、
仮のプレースホルダー機体をランプ手前のGround Area(赤側Start Zone付近)に
スポーンする。ランプ勾配・階段段差の走破性検証(移動シミュレーション)が目的。

使い方:
    ros2 launch nhk27_field gz_sim.launch.py
    # 別ターミナルでteleopするか、cmd_velへ直接publishして走行確認
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("nhk27_field")
    world_path = os.path.join(pkg_share, "worlds", "nhk27_field.sdf")
    xacro_path = os.path.join(pkg_share, "urdf", "placeholder_robot.urdf.xacro")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    robot_description = Command(["xacro ", xacro_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Start Zone(赤・TR用、フィールド南端の西角)付近にスポーン。西側のランプ/階段
    # (Y=-3.5付近)まで移動して走破性を確認できる位置。
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "nhk27_placeholder",
            "-topic", "robot_description",
            "-x", "-5.15", "-y", "-5.15", "-z", "0.05",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
