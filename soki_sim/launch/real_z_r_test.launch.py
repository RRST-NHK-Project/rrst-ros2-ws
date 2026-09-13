import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """z_joint/r_joint(motor1/motor2、ロボマスM2006+C610)単独の実機動作確認用launch。

    2026-08-31のz/r位置真値変更(CAN_HOST外付けENC1/ENC2 -> ROBOMAS内蔵ロータ
    エンコーダのCAN帰還)を実機で検証するためのlaunch。real_root_theta_test.launch.py
    (root_theta単独)と対になる、z/r単独版。real_axes_test.launch.pyと違い
    root_theta(CubeMars)側のMIT出力はここでは有効化しない
    (cubemars_joint_names=[]で実機出力オフ、既存のroot_theta較正・配線に影響を
    与えずz/rだけを切り分けて検証できるようにするため)。

    display.launch.py・real_root_theta_test.launch.py・real_axes_test.launch.pyと
    同時起動しないこと(いずれもtrajectory_follower_nodeを起動するため二重起動に
    なり衝突する)。

    起動するもの: ros2can(既定はPyQt5ウィンドウあり。ros2can_nogui引数参照)、
    real_joint_bridge_node(帰還確認。CubeMars側は
    root_theta/tip_thetaとも動かさない前提のためcubemars_root_theta_indexは
    yaml既定値のまま)、trajectory_follower_node(z/r実機出力あり、root_thetaは
    実機出力オフのままsoki_sim表示のみ)、homing_node(z/rの起動時ホーミング。
    自動開始はしない、start_homing_z/start_homing_r serviceで軸ごとに
    明示的に起動すること)、command_gui_node。
    use_joy:=true でjoy_node/joy_teleop_nodeも起動する(control_modeは自動的に
    'both'になる)。enable_buttonでデッドマンスイッチを指定できる
    (real_root_theta_test.launch.py参照、デフォルト-1=常時有効)。
    use_viz:=true でrobot_state_publisher/joint_state_publisher/rviz2も起動する。

    robomas_kp/robomas_kd/robomas_current_ffは要実機調整(M2006の電流上限1.0A基準で
    デフォルト値を決めてある。詳細はnote/hardware_mapping.txt「z_joint/r_jointの
    実機出力(RoboMas MITモード)」参照)。既定値は2026-09-09時点でsoki_sim/config/
    gains.json経由の実機調整済み値(Kp=0.5, Kd=0.1)に合わせてあり、
    z_max_velocity/r_max_velocity(既定0.036m/s)もhoming_nodeのホーミング速度と
    揃えてある(ユーザー指定:「r,zのゲインを最高速度がホーミングのときと
    同じくらいになるように」)。
    motor1_sign/motor2_sign(real_joint_bridge.yaml)は旧ENC1/ENC2較正時の値を
    そのまま流用しているだけなので、ロボマス内蔵エンコーダで符号が合っているか
    低速から要確認(note/hardware_mapping.txt「未確認事項」参照)。

    手順の目安:
      1. ros2can GUIでdevice_id=21(MODE_ROBOMAS)のtopic_passthroughをONにする
      2. motor1/motor2を手で少し回し、command_gui_nodeの「現在状態」表示の
         z_joint/r_jointが期待通りの符号で動くか確認(motor1_sign/motor2_sign)
      3. z/r原点センサ(SW1/SW2)の配線を確認しつつ、低速(既定30rpm)で
         /start_homing_z・/start_homing_r をそれぞれ呼び、z/rが正しく
         較正されるか確認(軸ごとに独立して実行できる)
      4. command_gui_node/joyから低速でz/rを動かし、MIT指令(robomas_kp/kd)の
         挙動を確認しながら徐々にゲインを上げる
    """
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')
    real_joint_bridge_yaml = os.path.join(pkg_share, 'config', 'real_joint_bridge.yaml')

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
                # trajectory_follower_node側のoutput_topic(下記)と一致させること。
                # root_theta(このlaunchでは実機出力なし)等、まだ実機帰還の無い軸は
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

    autotune_node = Node(
        package='soki_sim',
        executable='autotune_node',
        name='autotune_node',
        output='screen',
    )

    trajectory_follower_node = Node(
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
                # root_thetaはjoint_namesに含めるがcubemars_joint_names=[]のため
                # 実機出力はしない(soki_sim表示のみ、このlaunchではCubeMars側に
                # 触れない)。max_velocity/max_accelerationはjoint_names全要素分
                # 必要なため、root_theta分もダミーで入れておく(出力しないので安全)。
                'joint_names': ['root_theta_joint', 'z_joint', 'r_joint'],
                'max_velocity': ParameterValue(
                    [0.1, z_max_velocity, r_max_velocity], value_type=List[float]),
                'max_acceleration': ParameterValue(
                    [0.2, z_max_acceleration, r_max_acceleration], value_type=List[float]),
                'max_deceleration': ParameterValue(
                    [0.4, z_max_deceleration, r_max_deceleration], value_type=List[float]),
                'update_rate_hz': 50.0,
                'control_mode': control_mode,
                # sim表示(mixed_joint_states)はreal_joint_bridge_nodeの実測値を
                # 真値として使う(2026-09-07方針変更、real_root_theta_test.launch.py
                # 参照)。本ノードのpos_はMIT指令生成用の理想軌道でしかないため
                # 出力先を分離した。
                'output_topic': 'trajectory_target_joint_states',
                # 空配列はlaunch_rosが要素型を推定できずエラーになるため
                # (real_root_theta_test.launch.py等の単一要素配列と同じ理由)、
                # ParameterValueでList[str]と明示する。
                'cubemars_joint_names': ParameterValue([], value_type=List[str]),
                # z/rは常時実機出力を有効化する(このlaunchの目的そのものなのでトグルなし。
                # note/can_mapping.txt確認済みのdevice_id=21固定)。
                'robomas_device_id': 21,
                'robomas_kp': ParameterValue(robomas_kp, value_type=float),
                'robomas_kd': ParameterValue(robomas_kd, value_type=float),
                'robomas_current_ff': ParameterValue(robomas_current_ff, value_type=float),
            },
        ],
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

    # z/r上限・下限リミットスイッチの位置をrvizへ表示する(2026-09-09追加)。
    # real_joint_bridge_yamlのlimit_switch_marker_node節から実機のCAN配線・
    # 実測位置を読む(trajectory_follower_node節と同じ配線を指すよう値を
    # 一致させてあること、config/real_joint_bridge.yaml参照)。turret_link/
    # lift_linkにマーカーを置くためrviz表示と同じくuse_viz時のみ起動する。
    limit_switch_marker_node = Node(
        package='soki_sim',
        executable='limit_switch_marker_node',
        name='limit_switch_marker_node',
        parameters=[real_joint_bridge_yaml],
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
        autotune_node,
        trajectory_follower_node,
        command_gui_node,
        joy_node,
        joy_teleop_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        limit_switch_marker_node,
        rviz_node,
    ])
