from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare("nr26_r2_sim").find("nr26_r2_sim")
    urdf_file = os.path.join(pkg_share, "urdf", "nr26_r2.urdf.xacro")

    return LaunchDescription(
        [
            # Gazebo Harmonic 起動（空ワールド）
            ExecuteProcess(cmd=["gz", "sim", "-r", "empty.sdf"], output="screen"),
            # robot_state_publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": Command(["xacro ", urdf_file])}],
                output="screen",
            ),
            # ROS → Gazebo へロボットを生成
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "nr26_r2", "-topic", "robot_description"],
                output="screen",
            ),
        ]
    )
