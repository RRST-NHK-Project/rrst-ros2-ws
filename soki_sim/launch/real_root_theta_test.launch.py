import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """root_theta_joint(CubeMars AK40-10)の実機動作確認用launch。

    display.launch.pyはcubemars_*パラメータ未設定(実機出力オフ)のsim専用構成
    なので、実機確認時はこちらを使う(display.launch.pyと同時起動しないこと。
    trajectory_follower_nodeが二重起動になり衝突する)。

    起動するもの: ros2can GUI, real_joint_bridge_node(帰還確認),
    trajectory_follower_node(実機出力あり), command_gui_node。
    use_joy:=true でjoy_node/joy_teleop_nodeも起動する
    (この場合control_modeは自動的に'both'になる)。joy_teleop_nodeには
    enable_buttonでデッドマンスイッチ(デフォルト4=L1想定、実機コントローラで要確認)
    を渡す。ボタンを押している間だけ動き、離すと即座に停止する
    (2026-08-27、根本θが無操作で回転し続けた件を受けて追加。デッドマンスイッチ
    なしでの実機テストは非推奨)。
    use_viz:=true でrobot_state_publisher/joint_state_publisher/rviz2も起動する。

    device_id/motor_index/reduction/kp/kdは実機配線・note/hardware_mapping.txtの
    値に合わせて起動時に上書きすること(例: motor_index:=0 でM1配線に変更)。
    デフォルトは2026-08-27の動作確認時点の配線(root_theta=M2)・低ゲイン
    (Kp=5, Kd=0.5)。詳細はnote/command.txt参照。

    use_robomas:=true でz_joint/r_joint(motor1/motor2、ロボマスdevice_id=21、
    note/can_mapping.txt確認済み)へもMIT指令を送る(2026-08-29追加)。
    robomas_kp/robomas_kdは要実機調整・低ゲインから開始すること(M2006の電流
    上限1.0A(ros2can/firmware/.../config.hppのROBOMAS_MAX_CURRENT_A)基準で
    デフォルト値を決めてある。詳細はnote/hardware_mapping.txt「z_joint/r_jointの
    実機出力(RoboMas MITモード)」参照)。ホーミング未実施のままだとz/rの原点は
    未較正(生値)のままなので、先にhoming_nodeのstart_homingを実施すること。
    """
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')
    real_joint_bridge_yaml = os.path.join(pkg_share, 'config', 'real_joint_bridge.yaml')

    device_id_arg = DeclareLaunchArgument(
        'device_id', default_value='11',
        description='root_thetaのCubeMars(MODE_CUBEMARS)device_id')
    motor_index_arg = DeclareLaunchArgument(
        'motor_index', default_value='0',
        description='root_thetaのモータ番号(0-3=M1-M4)。2026-08-27時点はM2配線')
    reduction_arg = DeclareLaunchArgument(
        'reduction', default_value='4.666666666666667',
        description='外部減速比(112/24)。note/hardware_mapping.txt参照')
    kp_arg = DeclareLaunchArgument(
        'kp', default_value='5.0', description='MITモードKp(0-500)')
    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.5', description='MITモードKd(0-5)')
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity', default_value='0.1',
        description='root_thetaの最大速度[rad/s](安全のため低めから)')
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration', default_value='0.2',
        description='root_thetaの最大加速度[rad/s^2]')
    use_joy_arg = DeclareLaunchArgument(
        'use_joy', default_value='false',
        description='trueならjoy_node/joy_teleop_nodeも起動し、'
                    'control_modeを自動的にbothにする')
    enable_button_arg = DeclareLaunchArgument(
        'enable_button', default_value='-1',
        description='joy_teleop_nodeのデッドマンスイッチボタン番号。'
                    'このボタンを押している間だけ動く(離すとすぐ停止)。'
                    'デフォルト-1は常時有効(デッドマンスイッチなし)。'
                    '2026-08-27にデフォルト4(L1想定)を試したが、このコントローラの'
                    '実際のボタン配列と合わず手動操作が全く効かなくなったため-1に戻した。'
                    '有効にしたい場合は、先に ros2 topic echo /joy でボタンを押しながら'
                    '実際のindexを確認してから enable_button:=N を指定すること')
    use_viz_arg = DeclareLaunchArgument(
        'use_viz', default_value='false',
        description='trueならrobot_state_publisher/joint_state_publisher/rviz2も起動する')
    use_robomas_arg = DeclareLaunchArgument(
        'use_robomas', default_value='false',
        description='trueならz_joint/r_joint(motor1/motor2、ロボマスdevice_id=21)へも'
                    'MIT指令を送る(実機出力有効化)。falseならこれまで通りsoki_sim表示のみ')
    robomas_kp_arg = DeclareLaunchArgument(
        'robomas_kp', default_value='0.02',
        description='ロボマスMITモードKp[A/deg]。要実機調整、低ゲインから開始すること'
                    '(M2006の電流上限1.0A基準、誤差10degで0.2A程度になる想定値)')
    robomas_kd_arg = DeclareLaunchArgument(
        'robomas_kd', default_value='0.002',
        description='ロボマスMITモードKd[A/rpm]。要実機調整、低ゲインから開始すること')

    device_id = LaunchConfiguration('device_id')
    motor_index = LaunchConfiguration('motor_index')
    reduction = LaunchConfiguration('reduction')
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    max_velocity = LaunchConfiguration('max_velocity')
    max_acceleration = LaunchConfiguration('max_acceleration')
    use_joy = LaunchConfiguration('use_joy')
    use_viz = LaunchConfiguration('use_viz')
    enable_button = LaunchConfiguration('enable_button')
    use_robomas = LaunchConfiguration('use_robomas')
    robomas_kp = LaunchConfiguration('robomas_kp')
    robomas_kd = LaunchConfiguration('robomas_kd')

    # use_robomas:=falseならrobomas_device_id=0のまま(実機出力無効、trajectory_follower_node
    # 側のデフォルトと同じ)。trueなら21(note/can_mapping.txt確認済み)。
    robomas_device_id = PythonExpression(["21 if '", use_robomas, "' == 'true' else 0"])

    # use_joy:=trueならGUI/joy両方を受け付ける。falseならGUI専用のまま
    # (joy_teleop_nodeを起動しないなら'manual'を受け付けても無意味なため)。
    control_mode = PythonExpression(["'both' if '", use_joy, "' == 'true' else 'auto'"])

    ros2can_node = Node(
        package='ros2can',
        executable='ros2can',
        name='ros2can_gui',
        output='screen',
    )

    real_joint_bridge_node = Node(
        package='soki_sim',
        executable='real_joint_bridge_node',
        name='real_joint_bridge_node',
        output='screen',
        parameters=[
            real_joint_bridge_yaml,
            {'cubemars_root_theta_index': motor_index},
        ],
    )

    trajectory_follower_node = Node(
        package='soki_sim',
        executable='trajectory_follower_node',
        name='trajectory_follower_node',
        output='screen',
        parameters=[{
            # 単一要素配列は、bareなPythonリスト([x])で渡すとlaunch_rosに単一の
            # 文字列(concatenation)として解釈され、rclpy側の宣言型(DOUBLE_ARRAY等)
            # と衝突してノードが起動時に落ちる。ParameterValue(..., value_type=List[T])
            # で明示的に配列型として評価させること。
            # z_joint/r_jointは常時joint_namesに含める(soki_sim表示上のtrap追従は
            # use_robomasに関わらず有効。実機出力の有無はrobomas_device_idの方で
            # 制御する、trajectory_follower_node.py参照)。
            'joint_names': ['root_theta_joint', 'z_joint', 'r_joint'],
            'max_velocity': ParameterValue([max_velocity, 0.05, 0.05], value_type=List[float]),
            'max_acceleration': ParameterValue([max_acceleration, 0.1, 0.1], value_type=List[float]),
            'update_rate_hz': 50.0,
            'control_mode': control_mode,
            'cubemars_joint_names': ['root_theta_joint'],
            'cubemars_device_ids': ParameterValue([[device_id]], value_type=List[int]),
            'cubemars_motor_indices': ParameterValue([[motor_index]], value_type=List[int]),
            'cubemars_kp': ParameterValue([[kp]], value_type=List[float]),
            'cubemars_kd': ParameterValue([[kd]], value_type=List[float]),
            'cubemars_torque_ff': [0.0],
            'cubemars_reduction': ParameterValue([[reduction]], value_type=List[float]),
            'robomas_device_id': ParameterValue(robomas_device_id, value_type=int),
            'robomas_kp': ParameterValue(robomas_kp, value_type=float),
            'robomas_kd': ParameterValue(robomas_kd, value_type=float),
        }],
    )

    command_gui_node = Node(
        package='soki_sim',
        executable='command_gui_node',
        name='command_gui_node',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=IfCondition(use_joy),
    )

    joy_teleop_node = Node(
        package='soki_sim',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        parameters=[{'enable_button': ParameterValue(enable_button, value_type=int)}],
        condition=IfCondition(use_joy),
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(use_viz),
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['mixed_joint_states']}],
        condition=IfCondition(use_viz),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_viz),
    )

    return LaunchDescription([
        device_id_arg,
        motor_index_arg,
        reduction_arg,
        kp_arg,
        kd_arg,
        max_velocity_arg,
        max_acceleration_arg,
        use_joy_arg,
        use_viz_arg,
        enable_button_arg,
        use_robomas_arg,
        robomas_kp_arg,
        robomas_kd_arg,
        ros2can_node,
        real_joint_bridge_node,
        trajectory_follower_node,
        command_gui_node,
        joy_node,
        joy_teleop_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
