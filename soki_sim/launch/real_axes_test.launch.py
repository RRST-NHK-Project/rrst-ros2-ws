import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """root_theta_joint/z_joint/r_joint(手先θ=tip_theta_jointを除く軸)の実機動作確認用launch。

    real_root_theta_test.launch.py(root_theta単独確認用)をベースに、z/r
    (motor1/motor2、ロボマスdevice_id=21、note/can_mapping.txt確認済み)への
    MIT出力も常時有効にしたもの(2026-08-29追加)。tip_theta_jointはこのlaunchでは
    trajectory_follower_nodeからのMIT指令対象外のまま(cubemars_joint_namesに
    root_theta_jointしか含めない。2026-09-08方針変更でtip_theta_joint自体は
    ROBOMAS側(device_id=21のM3)へ移行済みのため、real_joint_bridge_nodeは
    本launchでも常時M3の帰還を使ってtip_theta_jointの表示を続ける。
    trajectory_follower_node側でtip_thetaへMIT指令を送りたい場合は
    real_all_axes_test.launch.pyを使うこと)。

    display.launch.py・real_root_theta_test.launch.pyと同時起動しないこと
    (いずれもtrajectory_follower_nodeを起動するため二重起動になり衝突する)。

    起動するもの: ros2can(既定はPyQt5ウィンドウあり。ros2can_nogui引数参照)、
    real_joint_bridge_node(帰還確認),
    trajectory_follower_node(root_theta+z/r実機出力あり), homing_node
    (z/rの起動時ホーミング。自動開始はしない、start_homing_z/start_homing_r
    serviceで軸ごとに明示的に起動すること)、command_gui_node。
    use_joy:=true でjoy_node/joy_teleop_nodeも起動する(control_modeは自動的に
    'both'になる)。enable_buttonでデッドマンスイッチを指定できる(デフォルト-1=
    常時有効。real_root_theta_test.launch.py参照、デッドマンスイッチなしでの
    実機テストは非推奨)。
    use_viz:=true でrobot_state_publisher/joint_state_publisher/rviz2も起動する。

    root_theta用のdevice_id/motor_index/reduction/kp/kdは実機配線に合わせて
    起動時に上書きすること。z/r用のrobomas_kp/robomas_kd/robomas_current_ffは
    要実機調整(M2006の電流上限1.0A基準でデフォルト値を決めてある。詳細は
    note/hardware_mapping.txt「z_joint/r_jointの実機出力(RoboMas MITモード)」
    参照)。既定値は2026-09-09時点でsoki_sim/config/gains.json経由の実機調整済み
    値(Kp=0.5, Kd=0.1)に合わせてあり、z_max_velocity/r_max_velocity(既定0.036m/s)
    もhoming_nodeのホーミング速度と揃えてある(ユーザー指定:「r,zのゲインを
    最高速度がホーミングのときと同じくらいになるように」)。

    z/rはホーミング未実施だと原点が未較正(生値)のままなので、起動後まず
    ros2can GUIでdevice_id=21(MODE_ROBOMAS)のtopic_passthroughをONにしてから
    /start_homing_z・/start_homing_r(いずれもstd_srvs/Trigger)を軸ごとに
    呼ぶこと(homing_node実行中はtrajectory_follower_node側のロボマス出力が
    自動的に一時停止される、note/command.txt参照)。
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
        'motor_index', default_value='1',
        description='root_thetaのモータ番号(0-3=M1-M4)。2026-09-08方針変更で'
                    '旧tip_theta側のAK40-10をCAN ID据え置き(M2)で転用したため'
                    '既定値は1(M2)')
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
    root_theta_max_deceleration_arg = DeclareLaunchArgument(
        'root_theta_max_deceleration', default_value='0.4',
        description='root_thetaの最大減速度[rad/s^2](既定はmax_accelerationの2倍。'
                    '停止時の応答性向上、trajectory_follower_node.py参照)')

    # ---- z/r (RoboMas) ----
    robomas_kp_arg = DeclareLaunchArgument(
        'robomas_kp', default_value='0.5',
        description='ロボマスMITモードKp[A/deg]。2026-09-09、ユーザー指定「r,zの'
                    'ゲインを最高速度がホーミングのときと同じくらいになるように」'
                    'を受け、以前は低ゲインから開始する初期値0.02のままだったのを'
                    '既にsoki_sim/config/gains.json経由で実機調整済みの値(0.5)に'
                    '合わせた(GUIの「ゲイン調整」タブから自動適用される値と、この'
                    'launch単体起動時の初期値が食い違っていたのを解消。M2006の'
                    '電流上限1.0A基準、誤差2degで飽和する強さ)')
    robomas_kd_arg = DeclareLaunchArgument(
        'robomas_kd', default_value='0.1',
        description='ロボマスMITモードKd[A/rpm]。2026-09-09、robomas_kpと同じ理由で'
                    'gains.jsonの実機調整済み値(0.1)に合わせた')
    robomas_current_ff_arg = DeclareLaunchArgument(
        'robomas_current_ff', default_value='0.0',
        description='ロボマスMITモードcurrent_ff[A](フィードフォワード電流)')
    z_max_velocity_arg = DeclareLaunchArgument(
        'z_max_velocity', default_value='0.036',
        description='z_jointの最大速度[m/s]。2026-09-09、ユーザー指定によりhoming_node'
                    'のホーミング速度(homing_velocity_rpm=30rpm、pulley_pitch_'
                    'diameter_mm=22.92、mix_k=0.5からz_dot=mix_k*2*(30rpm*pulley_'
                    'radius)≒0.036m/sと計算)と揃えた(以前は0.05で、既にこれより'
                    '速い設定だったが、MITゲインが低くこの上限まで実際には出て'
                    'いなかった。今回ゲインと合わせて速度上限も明示的にホーミング'
                    '基準に揃える)')
    z_max_acceleration_arg = DeclareLaunchArgument(
        'z_max_acceleration', default_value='0.072',
        description='z_jointの最大加速度[m/s^2](z_max_velocityの2倍のまま維持)')
    z_max_deceleration_arg = DeclareLaunchArgument(
        'z_max_deceleration', default_value='0.144',
        description='z_jointの最大減速度[m/s^2](既定はz_max_accelerationの2倍。'
                    '停止時の応答性向上、trajectory_follower_node.py参照)')
    r_max_velocity_arg = DeclareLaunchArgument(
        'r_max_velocity', default_value='0.036',
        description='r_jointの最大速度[m/s]。2026-09-09、z_max_velocityと同じ理由・'
                    '同じ計算(ホーミング速度基準)で揃えた')
    r_max_acceleration_arg = DeclareLaunchArgument(
        'r_max_acceleration', default_value='0.072',
        description='r_jointの最大加速度[m/s^2](r_max_velocityの2倍のまま維持)')
    r_max_deceleration_arg = DeclareLaunchArgument(
        'r_max_deceleration', default_value='0.144',
        description='r_jointの最大減速度[m/s^2](既定はr_max_accelerationの2倍。'
                    '停止時の応答性向上、trajectory_follower_node.py参照)')

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
    ros2can_nogui_arg = DeclareLaunchArgument(
        'ros2can_nogui', default_value='false',
        description='trueならros2canを--nogui(ターミナルダッシュボード、PyQt5ウィンドウ'
                    'なし)で起動する。ros2can自体は常に起動する。デフォルトfalse'
                    '(=PyQt5ウィンドウあり。real_all_axes_test.launch.pyと同じ既定。'
                    'ターミナルダッシュボードにしたい場合は ros2can_nogui:=true を指定する)')

    device_id = LaunchConfiguration('device_id')
    motor_index = LaunchConfiguration('motor_index')
    reduction = LaunchConfiguration('reduction')
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    root_theta_max_velocity = LaunchConfiguration('root_theta_max_velocity')
    root_theta_max_acceleration = LaunchConfiguration('root_theta_max_acceleration')
    root_theta_max_deceleration = LaunchConfiguration('root_theta_max_deceleration')
    robomas_kp = LaunchConfiguration('robomas_kp')
    robomas_kd = LaunchConfiguration('robomas_kd')
    robomas_current_ff = LaunchConfiguration('robomas_current_ff')
    z_max_velocity = LaunchConfiguration('z_max_velocity')
    z_max_acceleration = LaunchConfiguration('z_max_acceleration')
    z_max_deceleration = LaunchConfiguration('z_max_deceleration')
    r_max_velocity = LaunchConfiguration('r_max_velocity')
    r_max_acceleration = LaunchConfiguration('r_max_acceleration')
    r_max_deceleration = LaunchConfiguration('r_max_deceleration')
    use_joy = LaunchConfiguration('use_joy')
    use_viz = LaunchConfiguration('use_viz')
    enable_button = LaunchConfiguration('enable_button')

    # use_joy:=trueならGUI/joy両方を受け付ける。falseならGUI専用のまま
    control_mode = PythonExpression(["'both' if '", use_joy, "' == 'true' else 'auto'"])

    # ros2canは常に起動する。--noguiの有無だけをros2can_noguiで切り替える
    # (launch_ros.Nodeのargumentsは条件付きで一部だけ足すことができないため、
    # 同名ノードをIfCondition/UnlessConditionで排他的に2つ用意する定番パターン。
    # real_all_axes_test.launch.pyと同じ構成)。
    ros2can_nogui = LaunchConfiguration('ros2can_nogui')
    ros2can_gui_node = Node(
        package='ros2can',
        executable='ros2can',
        name='ros2can_gui',
        output='screen',
        condition=UnlessCondition(ros2can_nogui),
    )
    ros2can_nogui_node = Node(
        package='ros2can',
        executable='ros2can',
        name='ros2can_gui',
        output='screen',
        arguments=['--nogui'],
        condition=IfCondition(ros2can_nogui),
    )

    real_joint_bridge_node = Node(
        package='soki_sim',
        executable='real_joint_bridge_node',
        name='real_joint_bridge_node',
        output='screen',
        parameters=[
            real_joint_bridge_yaml,
            {
                'cubemars_root_theta_index': motor_index,
                # trajectory_follower_node側のoutput_topic(下記)と一致させること。
                # tip_theta(このlaunchでは対象外)等、まだ実機帰還の無い軸は
                # ここから理想軌道を転送してsim表示を動かし続ける
                # (note/hardware_mapping.txt「mixed_joint_statesの真値ソース」参照)。
                'fallback_topic': 'trajectory_target_joint_states',
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
        root_theta_decel = float(root_theta_max_deceleration.perform(context))
        z_vel = float(z_max_velocity.perform(context))
        z_accel = float(z_max_acceleration.perform(context))
        z_decel = float(z_max_deceleration.perform(context))
        r_vel = float(r_max_velocity.perform(context))
        r_accel = float(r_max_acceleration.perform(context))
        r_decel = float(r_max_deceleration.perform(context))

        return [Node(
            package='soki_sim',
            executable='trajectory_follower_node',
            name='trajectory_follower_node',
            output='screen',
            parameters=[{
                'joint_names': ['root_theta_joint', 'z_joint', 'r_joint'],
                'max_velocity': [root_theta_vel, z_vel, r_vel],
                'max_acceleration': [root_theta_accel, z_accel, r_accel],
                'max_deceleration': [root_theta_decel, z_decel, r_decel],
                'update_rate_hz': 50.0,
                'control_mode': control_mode,
                # sim表示(mixed_joint_states)はreal_joint_bridge_nodeの実測値を
                # 真値として使う(2026-09-07方針変更、real_root_theta_test.launch.py
                # 参照)。本ノードのpos_はMIT指令生成用の理想軌道でしかないため
                # 出力先を分離した。
                'output_topic': 'trajectory_target_joint_states',
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
        root_theta_max_deceleration_arg,
        robomas_kp_arg,
        robomas_kd_arg,
        robomas_current_ff_arg,
        z_max_velocity_arg,
        z_max_acceleration_arg,
        z_max_deceleration_arg,
        r_max_velocity_arg,
        r_max_acceleration_arg,
        r_max_deceleration_arg,
        use_joy_arg,
        use_viz_arg,
        ros2can_nogui_arg,
        enable_button_arg,
        ros2can_gui_node,
        ros2can_nogui_node,
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
