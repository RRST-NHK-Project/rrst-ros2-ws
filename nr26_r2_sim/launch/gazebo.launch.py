import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # パッケージ名
    package_name = "nr26_r2_sim"
    pkg_share = get_package_share_directory(package_name)

    # xacroの読み込み
    xacro_file = os.path.join(pkg_share, "urdf", "nr26_r2.urdf.xacro")
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw, "use_sim_time": True}],
    )

    # 2. Gazebo Sim (Jazzy用)
    # -r 引数は起動時にシミュレーションを自動再生します
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # 3. ロボットをGazeboにスポーン
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "nr26_r2",
            "-z",
            "0.2",  # 地上20cmから落とす
        ],
    )

    # 4. ROS-GZ Bridge (これがないと動きません)
    # ROS 2 の cmd_vel を Gazebo の物理エンジンに伝えます
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/skid_steer_drive_controller/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/skid_steer_drive_controller/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # 5. コントローラーのロード
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_skid_steer_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "skid_steer_drive_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            gz_sim,
            gz_spawn_entity,
            bridge,  # ブリッジを追加
            load_joint_state_broadcaster,
            load_skid_steer_drive_controller,
        ]
    )
