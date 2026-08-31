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

    起動するもの: ros2can GUI, real_joint_bridge_node(帰還確認。CubeMars側は
    root_theta/tip_thetaとも動かさない前提のためcubemars_root_theta_indexは
    yaml既定値のまま)、trajectory_follower_node(z/r実機出力あり、root_thetaは
    実機出力オフのままsoki_sim表示のみ)、homing_node(z/rの起動時ホーミング。
    自動開始はしない、start_homing serviceで明示的に起動すること)、
    command_gui_node。
    use_joy:=true でjoy_node/joy_teleop_nodeも起動する(control_modeは自動的に
    'both'になる)。enable_buttonでデッドマンスイッチを指定できる
    (real_root_theta_test.launch.py参照、デフォルト-1=常時有効)。
    use_viz:=true でrobot_state_publisher/joint_state_publisher/rviz2も起動する。

    robomas_kp/robomas_kd/robomas_current_ffは要実機調整・低ゲインから開始すること
    (M2006の電流上限1.0A基準でデフォルト値を決めてある。詳細は
    note/hardware_mapping.txt「z_joint/r_jointの実機出力(RoboMas MITモード)」参照)。
    motor1_sign/motor2_sign(real_joint_bridge.yaml)は旧ENC1/ENC2較正時の値を
    そのまま流用しているだけなので、ロボマス内蔵エンコーダで符号が合っているか
    低速から要確認(note/hardware_mapping.txt「未確認事項」参照)。

    手順の目安:
      1. ros2can GUIでdevice_id=21(MODE_ROBOMAS)のtopic_passthroughをONにする
      2. motor1/motor2を手で少し回し、command_gui_nodeの「現在状態」表示の
         z_joint/r_jointが期待通りの符号で動くか確認(motor1_sign/motor2_sign)
      3. z/r原点センサ(SW1/SW2)の配線を確認しつつ、低速(既定30rpm)で
         /start_homing を呼び、z/rが正しく較正されるか確認
      4. command_gui_node/joyから低速でz/rを動かし、MIT指令(robomas_kp/kd)の
         挙動を確認しながら徐々にゲインを上げる
    """
    pkg_share = get_package_share_directory('soki_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'soki_sim.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'soki_sim.rviz')
    real_joint_bridge_yaml = os.path.join(pkg_share, 'config', 'real_joint_bridge.yaml')

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
        parameters=[real_joint_bridge_yaml],
    )

    homing_node = Node(
        package='soki_sim',
        executable='homing_node',
        name='homing_node',
        output='screen',
        parameters=[real_joint_bridge_yaml],
    )

    trajectory_follower_node = Node(
        package='soki_sim',
        executable='trajectory_follower_node',
        name='trajectory_follower_node',
        output='screen',
        parameters=[{
            # root_thetaはjoint_namesに含めるがcubemars_joint_names=[]のため
            # 実機出力はしない(soki_sim表示のみ、このlaunchではCubeMars側に
            # 触れない)。max_velocity/max_accelerationはjoint_names全要素分
            # 必要なため、root_theta分もダミーで入れておく(出力しないので安全)。
            'joint_names': ['root_theta_joint', 'z_joint', 'r_joint'],
            'max_velocity': ParameterValue(
                [0.1, z_max_velocity, r_max_velocity], value_type=List[float]),
            'max_acceleration': ParameterValue(
                [0.2, z_max_acceleration, r_max_acceleration], value_type=List[float]),
            'update_rate_hz': 50.0,
            'control_mode': control_mode,
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
