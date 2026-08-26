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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        motor_mixer_node,
        rviz_node,
    ])
