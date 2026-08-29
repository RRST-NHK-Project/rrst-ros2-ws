import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='trueならjoint_state_publisher_guiのスライダーで関節を操作する')

    use_joy_arg = DeclareLaunchArgument(
        'use_joy', default_value='false',
        description='trueならjoyパッケージのjoy_node(ジョイスティック入力)と'
                    'joy_teleop_node(手動操作への変換)を起動する。'
                    '動作モード(自動専用/手動専用/併用)はcommand_gui_nodeの'
                    'GUIから切り替える(デフォルトはcontrol_mode=auto)')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # z_joint/r_jointはmotor1_joint/motor2_jointの差動機構で決まるため、
    # GUIスライダーではなくmotor_mixer_nodeが/mixed_joint_statesへ計算値を出し、
    # source_listでjoint_state_publisher(_gui)側にそれを取り込ませる。
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'source_list': ['mixed_joint_states']}],
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['mixed_joint_states']}],
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    motor_mixer_node = Node(
        package='soki_sim',
        executable='motor_mixer_node',
        name='motor_mixer_node',
        parameters=[{'mix_k': 0.5}],
    )

    # command_gui_nodeの目標値(/joint_targets)を台形速度プロファイルで滑らかに
    # 追従させ、/mixed_joint_statesへ出力する。max_velocity/max_accelerationは
    # 暫定値なので実機の動作速度に合わせて調整すること。cubemars_*系パラメータは
    # 実機のdevice_id/motor_indexが確定してから設定する(未設定時はsoki_sim表示のみ)。
    # z/rは元々0.05m/s・0.1m/s²だったが、joyでの手動ジョグがGUIクリック移動用の
    # この控えめな値では大きく遅延して感じたため引き上げた(soki_simはRVizの
    # 表示専用でcubemars_*未設定=実機出力は無効なので、この値を上げても実機側への
    # 影響はない)。
    trajectory_follower_node = Node(
        package='soki_sim',
        executable='trajectory_follower_node',
        name='trajectory_follower_node',
        parameters=[{
            'joint_names': ['root_theta_joint', 'z_joint', 'r_joint'],
            'max_velocity': [1.0, 0.2, 0.2],
            'max_acceleration': [2.0, 0.4, 0.4],
            'update_rate_hz': 50.0,
            # デフォルトは'both'(command_gui_node/joy_teleop_node併用)。
            # command_gui_nodeの「動作モード」パネルから'auto'/'manual'に切り替え可能
            # (2026-08-29、use_joy:=trueで起動してもGUI側で手動切り替えするまで
            # joy入力が無視されていた点を解消するためデフォルトを変更)。
            'control_mode': 'both',
        }],
    )

    # ジョイスティックによる手動操作(use_joy:=trueの時のみ起動)。
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    joy_teleop_node = Node(
        package='soki_sim',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        gui_arg,
        use_joy_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        motor_mixer_node,
        trajectory_follower_node,
        joy_node,
        joy_teleop_node,
        rviz_node,
    ])
