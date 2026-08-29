import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """root_theta_joint/z_joint/r_joint(手先θ=tip_theta_jointを除く軸)の実機動作確認用launch。

    real_root_theta_test.launch.py(root_theta単独確認用)をベースに、z/r
    (motor1/motor2、ロボマスdevice_id=21、note/can_mapping.txt確認済み)への
    MIT出力も常時有効にしたもの(2026-08-29追加)。tip_theta_jointは初回較正
    (tip_theta_offset_rad、note/hardware_mapping.txt参照)が未実施のため、
    このlaunchでは対象外のまま(cubemars_joint_namesにroot_theta_jointしか
    含めない。real_joint_bridge_node自体はCubeMars側M2の帰還を使って
    tip_theta_jointの表示は続けるが、trajectory_follower_nodeからのMIT指令は
    送らない。device 11のM2スロットは未更新のまま=0=速度ループ・target=0の
    安全なデフォルトで停止し続ける、ros2can/firmware/.../cubemars.cppの
    control_mode全ゼロ挙動と同じ)。

    display.launch.py・real_root_theta_test.launch.pyと同時起動しないこと
    (いずれもtrajectory_follower_nodeを起動するため二重起動になり衝突する)。

    起動するもの: ros2can GUI, real_joint_bridge_node(帰還確認),
    trajectory_follower_node(root_theta+z/r実機出力あり), homing_node
    (z/rの起動時ホーミング。自動開始はしない、start_homing serviceで明示的に
    起動すること)、command_gui_node。
    use_joy:=true でjoy_node/joy_teleop_nodeも起動する(control_modeは自動的に
    'both'になる)。enable_buttonでデッドマンスイッチを指定できる(デフォルト-1=
    常時有効。real_root_theta_test.launch.py参照、デッドマンスイッチなしでの
    実機テストは非推奨)。
    use_viz:=true でrobot_state_publisher/joint_state_publisher/rviz2も起動する。

    root_theta用のdevice_id/motor_index/reduction/kp/kdは実機配線に合わせて
    起動時に上書きすること。z/r用のrobomas_kp/robomas_kd/robomas_current_ffは
    要実機調整・低ゲインから開始すること(M2006の電流上限1.0A基準でデフォルト値を
    決めてある。詳細はnote/hardware_mapping.txt「z_joint/r_jointの実機出力
    (RoboMas MITモード)」参照)。

    z/rはホーミング未実施だと原点が未較正(生値)のままなので、起動後まず
    ros2can GUIでdevice_id=21(MODE_ROBOMAS)のtopic_passthroughをONにしてから
    /start_homing(std_srvs/Trigger)を呼ぶこと(homing_node実行中はtrajectory_
    follower_node側のロボマス出力が自動的に一時停止される、note/command.txt参照)。
    """
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')
    real_joint_bridge_yaml = os.path.join(pkg_share, 'config', 'real_joint_bridge.yaml')

    # ---- root_theta (CubeMars) ----
    device_id_arg = DeclareLaunchArgument(
        'device_id', default_value='11',
        description='root_thetaのCubeMars(MODE_CUBEMARS)device_id')
    motor_index_arg = DeclareLaunchArgument(
        'motor_index', default_value='0',
        description='root_thetaのモータ番号(0-3=M1-M4)')
    reduction_arg = DeclareLaunchArgument(
        'reduction', default_value='4.666666666666667',
        description='外部減速比(112/24)。note/hardware_mapping.txt参照')
    kp_arg = DeclareLaunchArgument(
        'kp', default_value='5.0', description='root_theta MITモードKp(0-500)')
    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.5', description='root_theta MITモードKd(0-5)')
    root_theta_max_velocity_arg = DeclareLaunchArgument(
        'root_theta_max_velocity', default_value='0.1',
        description='root_thetaの最大速度[rad/s](安全のため低めから)')
    root_theta_max_acceleration_arg = DeclareLaunchArgument(
        'root_theta_max_acceleration', default_value='0.2',
        description='root_thetaの最大加速度[rad/s^2]')

    # ---- z/r (RoboMas) ----
    robomas_kp_arg = DeclareLaunchArgument(
        'robomas_kp', default_value='0.02',
        description='ロボマスMITモードKp[A/deg]。要実機調整、低ゲインから開始すること'
                    '(M2006の電流上限1.0A基準、誤差10degで0.2A程度になる想定値)')
    robomas_kd_arg = DeclareLaunchArgument(
        'robomas_kd', default_value='0.002',
        description='ロボマスMITモードKd[A/rpm]。要実機調整、低ゲインから開始すること')
    robomas_current_ff_arg = DeclareLaunchArgument(
        'robomas_current_ff', default_value='0.0',
        description='ロボマスMITモードcurrent_ff[A](フィードフォワード電流)')
    z_max_velocity_arg = DeclareLaunchArgument(
        'z_max_velocity', default_value='0.05',
        description='z_jointの最大速度[m/s](安全のため低めから)')
    z_max_acceleration_arg = DeclareLaunchArgument(
        'z_max_acceleration', default_value='0.1',
        description='z_jointの最大加速度[m/s^2]')
    r_max_velocity_arg = DeclareLaunchArgument(
        'r_max_velocity', default_value='0.05',
        description='r_jointの最大速度[m/s](安全のため低めから)')
    r_max_acceleration_arg = DeclareLaunchArgument(
        'r_max_acceleration', default_value='0.1',
        description='r_jointの最大加速度[m/s^2]')

    # ---- joy/viz ----
    use_joy_arg = DeclareLaunchArgument(
        'use_joy', default_value='false',
        description='trueならjoy_node/joy_teleop_nodeも起動し、'
                    'control_modeを自動的にbothにする')
    enable_button_arg = DeclareLaunchArgument(
        'enable_button', default_value='-1',
        description='joy_teleop_nodeのデッドマンスイッチボタン番号。'
                    'デフォルト-1は常時有効(デッドマンスイッチなし)。'
                    '有効にしたい場合は、先に ros2 topic echo /joy でボタンを押しながら'
                    '実際のindexを確認してから enable_button:=N を指定すること')
    use_viz_arg = DeclareLaunchArgument(
        'use_viz', default_value='false',
        description='trueならrobot_state_publisher/joint_state_publisher/rviz2も起動する')

    device_id = LaunchConfiguration('device_id')
    motor_index = LaunchConfiguration('motor_index')
    reduction = LaunchConfiguration('reduction')
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    root_theta_max_velocity = LaunchConfiguration('root_theta_max_velocity')
    root_theta_max_acceleration = LaunchConfiguration('root_theta_max_acceleration')
    robomas_kp = LaunchConfiguration('robomas_kp')
    robomas_kd = LaunchConfiguration('robomas_kd')
    robomas_current_ff = LaunchConfiguration('robomas_current_ff')
    z_max_velocity = LaunchConfiguration('z_max_velocity')
    z_max_acceleration = LaunchConfiguration('z_max_acceleration')
    r_max_velocity = LaunchConfiguration('r_max_velocity')
    r_max_acceleration = LaunchConfiguration('r_max_acceleration')
    use_joy = LaunchConfiguration('use_joy')
    use_viz = LaunchConfiguration('use_viz')
    enable_button = LaunchConfiguration('enable_button')

    # use_joy:=trueならGUI/joy両方を受け付ける。falseならGUI専用のまま
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

    homing_node = Node(
        package='soki_sim',
        executable='homing_node',
        name='homing_node',
        output='screen',
        parameters=[real_joint_bridge_yaml],
    )

    def _make_trajectory_follower_node(context, *args, **kwargs):
        # max_velocity/max_accelerationは3つの独立したLaunchConfiguration
        # (root_theta/z/r)を1つの配列パラメータにまとめる必要があるが、
        # ParameterValue([a, b, c], value_type=List[T])は複数のsubstitutionを
        # 渡すと(単一要素の場合の[[x]]トリックと違い)全体を1つの文字列として
        # concatenateしてしまい、List[float]への変換に失敗する
        # (2026-08-29発覚: "Cannot convert value '0.10.050.05' to a list of
        # '<class 'float'>'")。OpaqueFunctionでcontext評価時に素のfloatへ
        # 解決してから、通常のPythonリストとして渡すことで回避する。
        root_theta_vel = float(root_theta_max_velocity.perform(context))
        root_theta_accel = float(root_theta_max_acceleration.perform(context))
        z_vel = float(z_max_velocity.perform(context))
        z_accel = float(z_max_acceleration.perform(context))
        r_vel = float(r_max_velocity.perform(context))
        r_accel = float(r_max_acceleration.perform(context))

        return [Node(
            package='soki_sim',
            executable='trajectory_follower_node',
            name='trajectory_follower_node',
            output='screen',
            parameters=[{
                'joint_names': ['root_theta_joint', 'z_joint', 'r_joint'],
                'max_velocity': [root_theta_vel, z_vel, r_vel],
                'max_acceleration': [root_theta_accel, z_accel, r_accel],
                'update_rate_hz': 50.0,
                'control_mode': control_mode,
                # tip_theta_jointは初回較正が済むまで対象外(ファイル冒頭docstring参照)。
                'cubemars_joint_names': ['root_theta_joint'],
                # 単一要素配列は、bareなPythonリスト([x])で渡すとlaunch_rosに単一の
                # 文字列(concatenation)として解釈され、rclpy側の宣言型(DOUBLE_ARRAY等)
                # と衝突してノードが起動時に落ちる。ParameterValue(..., value_type=
                # List[T])で明示的に配列型として評価させること
                # (real_root_theta_test.launch.py参照。上記のmax_velocity等と違い
                # 要素が1つだけなのでconcatenateされても結果は変わらず安全)。
                'cubemars_device_ids': ParameterValue([[device_id]], value_type=List[int]),
                'cubemars_motor_indices': ParameterValue([[motor_index]], value_type=List[int]),
                'cubemars_kp': ParameterValue([[kp]], value_type=List[float]),
                'cubemars_kd': ParameterValue([[kd]], value_type=List[float]),
                'cubemars_torque_ff': [0.0],
                'cubemars_reduction': ParameterValue([[reduction]], value_type=List[float]),
                # z/rは常時実機出力を有効化する(このlaunchの目的そのものなのでトグルなし。
                # note/can_mapping.txt確認済みのdevice_id=21固定)。
                'robomas_device_id': 21,
                'robomas_kp': ParameterValue(robomas_kp, value_type=float),
                'robomas_kd': ParameterValue(robomas_kd, value_type=float),
                'robomas_current_ff': ParameterValue(robomas_current_ff, value_type=float),
            }],
        )]

    trajectory_follower_node = OpaqueFunction(function=_make_trajectory_follower_node)

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
        root_theta_max_velocity_arg,
        root_theta_max_acceleration_arg,
        robomas_kp_arg,
        robomas_kd_arg,
        robomas_current_ff_arg,
        z_max_velocity_arg,
        z_max_acceleration_arg,
        r_max_velocity_arg,
        r_max_acceleration_arg,
        use_joy_arg,
        use_viz_arg,
        enable_button_arg,
        ros2can_node,
        real_joint_bridge_node,
        homing_node,
        trajectory_follower_node,
        command_gui_node,
        joy_node,
        joy_teleop_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
