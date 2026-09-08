import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
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
    ホーミング。自動開始はしない、start_homing_z/start_homing_r serviceで
    軸ごとに明示的に起動すること)、
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

    root_theta用のdevice_id/motor_index/reduction/kp/kdは実機配線に合わせて
    起動時に上書きすること(note/hardware_mapping.txt参照)。

    tip_thetaは2026-09-08方針変更でCubeMars AK40-10からRoboMas M2006(z/rと
    同一device_id=21のM3)へ移行した(root_theta側のAK40-10が電源逆接続で焼損した
    ため、無事だったtip_theta側のAK40-10をroot_thetaへ転用した経緯。
    note/hardware_mapping.txt参照)。z/rのような差動ミックスではなく単独直接駆動
    (robomas_tip_theta_kp/kdはz/rのrobomas_kp/kdと同じA/deg・A/rpmスケール)。
    原点センサが無いため、電源投入前に機構原点(0deg)へ手で合わせておくこと
    (tip_theta_offset_radは微調整のみに使う。M2006内蔵エンコーダの起動時
    リセット値をそのまま原点として使うため、root_thetaのようなSet Origin
    コマンドやホーミング処理は不要)。

    z/r用のrobomas_kp/robomas_kd/robomas_current_ffは要実機調整(M2006の電流上限
    1.0A基準でデフォルト値を決めてある。詳細はnote/hardware_mapping.txt
    「z_joint/r_jointの実機出力(RoboMas MITモード)」参照)。既定値は2026-09-09
    時点でsoki_sim/config/gains.json経由の実機調整済み値(Kp=0.5, Kd=0.1)に
    合わせてあり、z_max_velocity/r_max_velocity(既定0.036m/s)もhoming_nodeの
    ホーミング速度と揃えてある(ユーザー指定:「r,zのゲインを最高速度が
    ホーミングのときと同じくらいになるように」)。z/rはホーミング未実施だと
    原点が未較正(生値)のままなので、起動後まず
    ros2can GUIでdevice_id=21(MODE_ROBOMAS)のtopic_passthroughをONにしてから
    /start_homing_z・/start_homing_r(いずれもstd_srvs/Trigger)を軸ごとに
    呼ぶこと(homing_node実行中はtrajectory_follower_node側のロボマス出力が
    自動的に一時停止される、note/command.txt参照)。
    """
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')
    real_joint_bridge_yaml = os.path.join(pkg_share, 'config', 'real_joint_bridge.yaml')
    hand_yaml = os.path.join(pkg_share, 'config', 'hand.yaml')

    # ---- root_theta (CubeMars) ----
    cubemars_device_id_arg = DeclareLaunchArgument(
        'cubemars_device_id', default_value='11',
        description='root_thetaのCubeMars(MODE_CUBEMARS)device_id')
    root_theta_motor_index_arg = DeclareLaunchArgument(
        'root_theta_motor_index', default_value='1',
        description='root_thetaのモータ番号(0-3=M1-M4)。2026-09-08方針変更で'
                    '旧tip_theta側のAK40-10をCAN ID据え置き(M2)で転用したため'
                    '既定値は1(M2)')
    root_theta_reduction_arg = DeclareLaunchArgument(
        'root_theta_reduction', default_value='4.666666666666667',
        description='root_theta外部減速比(112/24)。note/hardware_mapping.txt参照')
    root_theta_kp_arg = DeclareLaunchArgument(
        'root_theta_kp', default_value='5.0', description='root_theta MITモードKp(0-500)')
    root_theta_kd_arg = DeclareLaunchArgument(
        'root_theta_kd', default_value='0.5', description='root_theta MITモードKd(0-5)')

    # ---- tip_theta (RoboMas M2006、z/rと同じdevice_idのM3。2026-09-08方針変更で
    # CubeMarsから移行。z/rのrobomas_kp/kdと同じA/deg・A/rpmスケール) ----
    robomas_tip_theta_index_arg = DeclareLaunchArgument(
        'robomas_tip_theta_index', default_value='2',
        description='tip_thetaのRoboMasモータ番号(0-3=M1-M4、既定2=M3)')
    tip_theta_reduction_arg = DeclareLaunchArgument(
        'tip_theta_reduction', default_value='1.4',
        description='tip_theta外部減速比(28T/20T)。note/hardware_mapping.txt参照')
    tip_theta_sign_arg = DeclareLaunchArgument(
        'tip_theta_sign', default_value='1.0', description='tip_theta回転方向(±1)')
    tip_theta_offset_rad_arg = DeclareLaunchArgument(
        'tip_theta_offset_rad', default_value='0.0',
        description='tip_theta原点オフセット[rad]。原点センサが無いため、電源投入前に'
                    '機構原点(0deg)へ手で合わせておくのが基本で、これは微調整のみに使う')
    tip_theta_kp_arg = DeclareLaunchArgument(
        'tip_theta_kp', default_value='0.02',
        description='tip_theta MITモードKp[A/deg]。要実機調整、低ゲインから開始すること'
                    '(robomas_kpと同じスケール)')
    tip_theta_kd_arg = DeclareLaunchArgument(
        'tip_theta_kd', default_value='0.002',
        description='tip_theta MITモードKd[A/rpm]。要実機調整、低ゲインから開始すること'
                    '(robomas_kdと同じスケール)')
    tip_theta_current_ff_arg = DeclareLaunchArgument(
        'tip_theta_current_ff', default_value='0.0',
        description='tip_theta MITモードcurrent_ff[A](フィードフォワード電流)')
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
    tip_theta_max_velocity_arg = DeclareLaunchArgument(
        'tip_theta_max_velocity', default_value='0.1',
        description='tip_thetaの最大速度[rad/s](安全のため低めから)')
    tip_theta_max_acceleration_arg = DeclareLaunchArgument(
        'tip_theta_max_acceleration', default_value='0.2',
        description='tip_thetaの最大加速度[rad/s^2]')
    tip_theta_max_deceleration_arg = DeclareLaunchArgument(
        'tip_theta_max_deceleration', default_value='0.4',
        description='tip_thetaの最大減速度[rad/s^2](既定はmax_accelerationの2倍。'
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
                    'なし)で起動する。ros2can自体は常に起動する(command_gui_nodeの'
                    '「実機セットアップ」パネルのチェックボックスで切替可能、2026-09-07'
                    '追加、デフォルトfalseで従来通りGUI)')
    launch_gui_arg = DeclareLaunchArgument(
        'launch_gui', default_value='true',
        description='falseならcommand_gui_nodeを起動しない。command_gui_node自身が'
                    '本launchを起動する場合(統合操作タブの「試合前セットアップ」)に、'
                    'GUIが二重に立ち上がるのを防ぐために使う')

    cubemars_device_id = LaunchConfiguration('cubemars_device_id')
    root_theta_motor_index = LaunchConfiguration('root_theta_motor_index')
    root_theta_reduction = LaunchConfiguration('root_theta_reduction')
    root_theta_kp = LaunchConfiguration('root_theta_kp')
    root_theta_kd = LaunchConfiguration('root_theta_kd')
    robomas_tip_theta_index = LaunchConfiguration('robomas_tip_theta_index')
    tip_theta_reduction = LaunchConfiguration('tip_theta_reduction')
    tip_theta_sign = LaunchConfiguration('tip_theta_sign')
    tip_theta_offset_rad = LaunchConfiguration('tip_theta_offset_rad')
    tip_theta_kp = LaunchConfiguration('tip_theta_kp')
    tip_theta_kd = LaunchConfiguration('tip_theta_kd')
    tip_theta_current_ff = LaunchConfiguration('tip_theta_current_ff')
    root_theta_max_velocity = LaunchConfiguration('root_theta_max_velocity')
    root_theta_max_acceleration = LaunchConfiguration('root_theta_max_acceleration')
    root_theta_max_deceleration = LaunchConfiguration('root_theta_max_deceleration')
    tip_theta_max_velocity = LaunchConfiguration('tip_theta_max_velocity')
    tip_theta_max_acceleration = LaunchConfiguration('tip_theta_max_acceleration')
    tip_theta_max_deceleration = LaunchConfiguration('tip_theta_max_deceleration')
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
    ros2can_nogui = LaunchConfiguration('ros2can_nogui')
    enable_button = LaunchConfiguration('enable_button')
    launch_gui = LaunchConfiguration('launch_gui')

    # use_joy:=trueならGUI/joy両方を受け付ける。falseならGUI専用のまま
    control_mode = PythonExpression(["'both' if '", use_joy, "' == 'true' else 'auto'"])

    # ros2canは常に起動する。--noguiの有無だけをros2can_noguiで切り替える
    # (launch_ros.Nodeのargumentsは条件付きで一部だけ足すことができないため、
    # 同名ノードをIfCondition/UnlessConditionで排他的に2つ用意する定番パターン)。
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
                'cubemars_device_id': cubemars_device_id,
                'cubemars_root_theta_index': root_theta_motor_index,
                'robomas_tip_theta_index': robomas_tip_theta_index,
                # trajectory_follower_node側のoutput_topic(下記)と一致させること。
                # 起動直後、まだ実機帰還が届いていない軸はここから理想軌道を転送して
                # sim表示を動かし続ける(note/hardware_mapping.txt
                # 「mixed_joint_statesの真値ソース」参照)。
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

    autotune_node = Node(
        package='soki_sim',
        executable='autotune_node',
        name='autotune_node',
        output='screen',
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
        root_theta_decel = float(root_theta_max_deceleration.perform(context))
        tip_theta_vel = float(tip_theta_max_velocity.perform(context))
        tip_theta_accel = float(tip_theta_max_acceleration.perform(context))
        tip_theta_decel = float(tip_theta_max_deceleration.perform(context))
        z_vel = float(z_max_velocity.perform(context))
        z_accel = float(z_max_acceleration.perform(context))
        z_decel = float(z_max_deceleration.perform(context))
        r_vel = float(r_max_velocity.perform(context))
        r_accel = float(r_max_acceleration.perform(context))
        r_decel = float(r_max_deceleration.perform(context))

        device_id = int(cubemars_device_id.perform(context))
        root_index = int(root_theta_motor_index.perform(context))
        root_kp = float(root_theta_kp.perform(context))
        root_kd = float(root_theta_kd.perform(context))
        root_reduction = float(root_theta_reduction.perform(context))
        robomas_tip_index = int(robomas_tip_theta_index.perform(context))
        tip_reduction = float(tip_theta_reduction.perform(context))
        tip_sign = float(tip_theta_sign.perform(context))
        tip_offset = float(tip_theta_offset_rad.perform(context))
        tip_kp = float(tip_theta_kp.perform(context))
        tip_kd = float(tip_theta_kd.perform(context))
        tip_current_ff = float(tip_theta_current_ff.perform(context))

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
                    'max_deceleration': [root_theta_decel, tip_theta_decel, z_decel, r_decel],
                    'update_rate_hz': 50.0,
                    'control_mode': control_mode,
                    # sim表示(mixed_joint_states)はreal_joint_bridge_nodeの実測値を
                    # 真値として使う(2026-09-07方針変更)。本ノードのpos_はMIT指令
                    # 生成用の理想軌道でしかなく、実機の追従遅れ次第でmixed_joint_
                    # statesに直接publishすると実測値と競合してsimが実機を置き去りに
                    # したように見える問題があったため、出力先を分離した。
                    'output_topic': 'trajectory_target_joint_states',
                    # 4軸ともCubeMars/RoboMasへのMIT実機出力を常時有効化する
                    # (本launchの目的そのものなのでトグルなし)。
                    'cubemars_joint_names': ['root_theta_joint'],
                    'cubemars_device_ids': [device_id],
                    'cubemars_motor_indices': [root_index],
                    'cubemars_kp': [root_kp],
                    'cubemars_kd': [root_kd],
                    'cubemars_torque_ff': [0.0],
                    'cubemars_reduction': [root_reduction],
                    # note/can_mapping.txt確認済みのdevice_id=21固定。
                    'robomas_device_id': 21,
                    'robomas_kp': float(robomas_kp.perform(context)),
                    'robomas_kd': float(robomas_kd.perform(context)),
                    'robomas_current_ff': float(robomas_current_ff.perform(context)),
                    # tip_theta(M3、2026-09-08新規): z/rと違いミックス無しの単独直接駆動。
                    'robomas_tip_theta_index': robomas_tip_index,
                    'robomas_tip_theta_joint': 'tip_theta_joint',
                    'tip_theta_reduction': tip_reduction,
                    'tip_theta_sign': tip_sign,
                    'tip_theta_offset_rad': tip_offset,
                    'robomas_tip_theta_kp': tip_kp,
                    'robomas_tip_theta_kd': tip_kd,
                    'robomas_tip_theta_current_ff': tip_current_ff,
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
        root_theta_reduction_arg,
        root_theta_kp_arg,
        root_theta_kd_arg,
        robomas_tip_theta_index_arg,
        tip_theta_reduction_arg,
        tip_theta_sign_arg,
        tip_theta_offset_rad_arg,
        tip_theta_kp_arg,
        tip_theta_kd_arg,
        tip_theta_current_ff_arg,
        root_theta_max_velocity_arg,
        root_theta_max_acceleration_arg,
        root_theta_max_deceleration_arg,
        tip_theta_max_velocity_arg,
        tip_theta_max_acceleration_arg,
        tip_theta_max_deceleration_arg,
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
        launch_gui_arg,
        ros2can_gui_node,
        ros2can_nogui_node,
        real_joint_bridge_node,
        homing_node,
        autotune_node,
        hand_node,
        trajectory_follower_node,
        command_gui_node,
        joy_node,
        joy_teleop_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
