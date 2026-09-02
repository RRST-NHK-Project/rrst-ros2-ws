import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """4軸(root_theta_joint/tip_theta_joint/z_joint/r_joint)すべての実機動作確認用launch。

    real_axes_test.launch.py(tip_theta_jointを除く3軸)をベースに、tip_theta_joint
    (CubeMars device_id=11のM2、note/hardware_mapping.txt参照)へのMIT出力も
    常時有効にしたもの(2026-08-31追加)。本番(競技当日)にそのまま使うことを
    想定しており、real_axes_test.launch.pyと違い軸を対象外にする機能はない
    (全軸を常時有効化する)。

    display.launch.py・real_root_theta_test.launch.py・real_axes_test.launch.pyと
    同時起動しないこと(いずれもtrajectory_follower_nodeを起動するため二重起動に
    なり衝突する)。

    起動するもの: ros2can GUI, real_joint_bridge_node(帰還確認、4軸とも
    ROBOMAS/CubeMarsの内蔵エンコーダ帰還を真値として使う。z/rはRoboMas内蔵
    エンコーダに統一済み、2026-08-31方針転換。note/hardware_mapping.txt参照)、
    trajectory_follower_node(4軸とも実機出力あり)、homing_node(z/rの起動時
    ホーミング。自動開始はしない、start_homing serviceで明示的に起動すること)、
    hand_node(吸着ハンド。吸着パッド展開/収納・ワークピッチ変更のTriggerサービスを
    提供、config/hand.yaml参照。実機配線未確認のうちはdevice_id=0のまま何もしない)、
    command_gui_node。
    use_joy:=true でjoy_node/joy_teleop_nodeも起動する(control_modeは自動的に
    'both'になる)。enable_buttonでデッドマンスイッチを指定できる(デフォルト-1=
    常時有効。real_root_theta_test.launch.py参照、デッドマンスイッチなしでの
    実機テストは非推奨)。
    use_viz:=true でrobot_state_publisher/joint_state_publisher/rviz2も起動する。
    launch_gui:=false でcommand_gui_nodeを起動しない(command_gui_node自身が
    本launchを起動する場合に、GUIの二重起動を防ぐために使う。デフォルトtrueで
    従来通りcommand_gui_nodeも起動する)。

    root_theta/tip_theta用のdevice_id/motor_index/reduction/kp/kdは実機配線に
    合わせて起動時に上書きすること(root_theta/tip_thetaは同一CubeMarsデバイス
    (device_id共通)上のM1/M2として配線されている前提、note/hardware_mapping.txt
    参照)。tip_thetaはroot_thetaと異なりCubeMars本体のSet Originコマンドを
    使わず、real_joint_bridge.yamlのtip_theta_offset_radで原点較正する
    (note/hardware_mapping.txt「root_theta/tip_thetaの原点初期化」参照)。
    初回較正がまだなら、原点センサ位置で_unwrapped値を読みtip_theta_offset_radを
    yamlへ書き込んでおくこと(でないとtip_thetaの位置がズレたまま動く)。

    z/r用のrobomas_kp/robomas_kd/robomas_current_ffは要実機調整・低ゲインから
    開始すること(M2006の電流上限1.0A基準でデフォルト値を決めてある。詳細は
    note/hardware_mapping.txt「z_joint/r_jointの実機出力(RoboMas MITモード)」
    参照)。z/rはホーミング未実施だと原点が未較正(生値)のままなので、起動後まず
    ros2can GUIでdevice_id=21(MODE_ROBOMAS)のtopic_passthroughをONにしてから
    /start_homing(std_srvs/Trigger)を呼ぶこと(homing_node実行中はtrajectory_
    follower_node側のロボマス出力が自動的に一時停止される、note/command.txt参照)。
    """
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')
    real_joint_bridge_yaml = os.path.join(pkg_share, 'config', 'real_joint_bridge.yaml')
    hand_yaml = os.path.join(pkg_share, 'config', 'hand.yaml')

    # ---- root_theta / tip_theta (CubeMars、同一device_id上のM1/M2) ----
    cubemars_device_id_arg = DeclareLaunchArgument(
        'cubemars_device_id', default_value='11',
        description='root_theta/tip_theta共通のCubeMars(MODE_CUBEMARS)device_id')
    root_theta_motor_index_arg = DeclareLaunchArgument(
        'root_theta_motor_index', default_value='0',
        description='root_thetaのモータ番号(0-3=M1-M4)')
    tip_theta_motor_index_arg = DeclareLaunchArgument(
        'tip_theta_motor_index', default_value='1',
        description='tip_thetaのモータ番号(0-3=M1-M4)')
    root_theta_reduction_arg = DeclareLaunchArgument(
        'root_theta_reduction', default_value='4.666666666666667',
        description='root_theta外部減速比(112/24)。note/hardware_mapping.txt参照')
    tip_theta_reduction_arg = DeclareLaunchArgument(
        'tip_theta_reduction', default_value='1.4',
        description='tip_theta外部減速比(28T/20T)。note/hardware_mapping.txt参照')
    root_theta_kp_arg = DeclareLaunchArgument(
        'root_theta_kp', default_value='5.0', description='root_theta MITモードKp(0-500)')
    root_theta_kd_arg = DeclareLaunchArgument(
        'root_theta_kd', default_value='0.5', description='root_theta MITモードKd(0-5)')
    tip_theta_kp_arg = DeclareLaunchArgument(
        'tip_theta_kp', default_value='5.0',
        description='tip_theta MITモードKp(0-500)。要実機調整、低ゲインから開始すること')
    tip_theta_kd_arg = DeclareLaunchArgument(
        'tip_theta_kd', default_value='0.5',
        description='tip_theta MITモードKd(0-5)。要実機調整、低ゲインから開始すること')
    root_theta_max_velocity_arg = DeclareLaunchArgument(
        'root_theta_max_velocity', default_value='0.1',
        description='root_thetaの最大速度[rad/s](安全のため低めから)')
    root_theta_max_acceleration_arg = DeclareLaunchArgument(
        'root_theta_max_acceleration', default_value='0.2',
        description='root_thetaの最大加速度[rad/s^2]')
    tip_theta_max_velocity_arg = DeclareLaunchArgument(
        'tip_theta_max_velocity', default_value='0.1',
        description='tip_thetaの最大速度[rad/s](安全のため低めから)')
    tip_theta_max_acceleration_arg = DeclareLaunchArgument(
        'tip_theta_max_acceleration', default_value='0.2',
        description='tip_thetaの最大加速度[rad/s^2]')

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
    launch_gui_arg = DeclareLaunchArgument(
        'launch_gui', default_value='true',
        description='falseならcommand_gui_nodeを起動しない。command_gui_node自身が'
                    '本launchを起動する場合(統合操作タブの「試合前セットアップ」)に、'
                    'GUIが二重に立ち上がるのを防ぐために使う')

    cubemars_device_id = LaunchConfiguration('cubemars_device_id')
    root_theta_motor_index = LaunchConfiguration('root_theta_motor_index')
    tip_theta_motor_index = LaunchConfiguration('tip_theta_motor_index')
    root_theta_reduction = LaunchConfiguration('root_theta_reduction')
    tip_theta_reduction = LaunchConfiguration('tip_theta_reduction')
    root_theta_kp = LaunchConfiguration('root_theta_kp')
    root_theta_kd = LaunchConfiguration('root_theta_kd')
    tip_theta_kp = LaunchConfiguration('tip_theta_kp')
    tip_theta_kd = LaunchConfiguration('tip_theta_kd')
    root_theta_max_velocity = LaunchConfiguration('root_theta_max_velocity')
    root_theta_max_acceleration = LaunchConfiguration('root_theta_max_acceleration')
    tip_theta_max_velocity = LaunchConfiguration('tip_theta_max_velocity')
    tip_theta_max_acceleration = LaunchConfiguration('tip_theta_max_acceleration')
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
    launch_gui = LaunchConfiguration('launch_gui')

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
            {
                'cubemars_device_id': cubemars_device_id,
                'cubemars_root_theta_index': root_theta_motor_index,
                'cubemars_tip_theta_index': tip_theta_motor_index,
            },
        ],
    )

    homing_node = Node(
        package='soki_sim',
        executable='homing_node',
        name='homing_node',
        output='screen',
        parameters=[real_joint_bridge_yaml],
    )

    hand_node = Node(
        package='soki_sim',
        executable='hand_node',
        name='hand_node',
        output='screen',
        parameters=[hand_yaml],
    )

    def _make_trajectory_follower_node(context, *args, **kwargs):
        # 複数のLaunchConfigurationを1つの配列パラメータにまとめる際、bareな
        # Pythonリスト([a, b])で渡すとlaunch_rosが全体を1つの文字列として
        # concatenateしてしまいList[float]への変換に失敗する
        # (real_axes_test.launch.py参照、2026-08-29発覚)。OpaqueFunctionで
        # context評価時に素のfloatへ解決してから渡すことで回避する。
        root_theta_vel = float(root_theta_max_velocity.perform(context))
        root_theta_accel = float(root_theta_max_acceleration.perform(context))
        tip_theta_vel = float(tip_theta_max_velocity.perform(context))
        tip_theta_accel = float(tip_theta_max_acceleration.perform(context))
        z_vel = float(z_max_velocity.perform(context))
        z_accel = float(z_max_acceleration.perform(context))
        r_vel = float(r_max_velocity.perform(context))
        r_accel = float(r_max_acceleration.perform(context))

        device_id = int(cubemars_device_id.perform(context))
        root_index = int(root_theta_motor_index.perform(context))
        tip_index = int(tip_theta_motor_index.perform(context))
        root_kp = float(root_theta_kp.perform(context))
        root_kd = float(root_theta_kd.perform(context))
        tip_kp = float(tip_theta_kp.perform(context))
        tip_kd = float(tip_theta_kd.perform(context))
        root_reduction = float(root_theta_reduction.perform(context))
        tip_reduction = float(tip_theta_reduction.perform(context))

        return [Node(
            package='soki_sim',
            executable='trajectory_follower_node',
            name='trajectory_follower_node',
            output='screen',
            # real_joint_bridge_yamlを先に読ませ、z/r上限・下限リミットスイッチ
            # (*_limit_switch_*)をyaml側から供給する(GUI「z/r安全停止センサ配線」
            # パネルで編集可能にするため)。同名キーは後に来る辞書側が勝つが、
            # このyamlのtrajectory_follower_node節にはlimit_switch系しか無いため
            # 以下の値と衝突しない。
            parameters=[
                real_joint_bridge_yaml,
                {
                    'joint_names': ['root_theta_joint', 'tip_theta_joint', 'z_joint', 'r_joint'],
                    'max_velocity': [root_theta_vel, tip_theta_vel, z_vel, r_vel],
                    'max_acceleration': [root_theta_accel, tip_theta_accel, z_accel, r_accel],
                    'update_rate_hz': 50.0,
                    'control_mode': control_mode,
                    # 4軸ともCubeMars/RoboMasへのMIT実機出力を常時有効化する
                    # (本launchの目的そのものなのでトグルなし)。
                    'cubemars_joint_names': ['root_theta_joint', 'tip_theta_joint'],
                    'cubemars_device_ids': [device_id, device_id],
                    'cubemars_motor_indices': [root_index, tip_index],
                    'cubemars_kp': [root_kp, tip_kp],
                    'cubemars_kd': [root_kd, tip_kd],
                    'cubemars_torque_ff': [0.0, 0.0],
                    'cubemars_reduction': [root_reduction, tip_reduction],
                    # note/can_mapping.txt確認済みのdevice_id=21固定。
                    'robomas_device_id': 21,
                    'robomas_kp': float(robomas_kp.perform(context)),
                    'robomas_kd': float(robomas_kd.perform(context)),
                    'robomas_current_ff': float(robomas_current_ff.perform(context)),
                },
            ],
        )]

    trajectory_follower_node = OpaqueFunction(function=_make_trajectory_follower_node)

    command_gui_node = Node(
        package='soki_sim',
        executable='command_gui_node',
        name='command_gui_node',
        condition=IfCondition(launch_gui),
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
        cubemars_device_id_arg,
        root_theta_motor_index_arg,
        tip_theta_motor_index_arg,
        root_theta_reduction_arg,
        tip_theta_reduction_arg,
        root_theta_kp_arg,
        root_theta_kd_arg,
        tip_theta_kp_arg,
        tip_theta_kd_arg,
        root_theta_max_velocity_arg,
        root_theta_max_acceleration_arg,
        tip_theta_max_velocity_arg,
        tip_theta_max_acceleration_arg,
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
        launch_gui_arg,
        ros2can_node,
        real_joint_bridge_node,
        homing_node,
        hand_node,
        trajectory_follower_node,
        command_gui_node,
        joy_node,
        joy_teleop_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
