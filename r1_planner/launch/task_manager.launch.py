from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    param_specs = [
        ("initial_state_code", "0", int),
        ("initial_color_code", "-1", int),
        ("initial_mff_cell", "0", int),
        ("initial_transition_mode", "0", int),
        ("auto_transition_period_ms", "3000", int),
        ("command_topic", "r1/task_command", str),
        ("state_topic", "r1/task_state", str),
        ("color_topic", "r1/task_color", str),
        ("cell_topic", "r1/task_cell", str),
        ("transition_mode_topic", "r1/task_transition_mode", str),
        ("state_sequence_topic", "r1/task_state_sequence", str),
        ("state_pose_topic", "r1/task_state_pose", str),
        ("state_mode_topic", "r1/task_state_mode", str),
        ("state_odom_reset_topic", "r1/task_state_odom_reset", str),
        ("state_wait_topic", "r1/task_state_wait_ms", str),
        ("auto_send_enabled_topic", "r1/task_auto_send_enabled", str),
        ("initial_auto_send_enabled", "true", bool),
        ("status_topic", "r1/task_status", str),
        ("status_text_topic", "r1/task_status_text", str),
        ("auto_drive_target_topic", "r1_autodrive_cmd", str),
        ("drive_mode_cmd_topic", "r1_drive_mode_cmd", str),
        ("mff_path_topic", "r1/task_mff_path", str),
        ("mff_path_advance_topic", "r1/task_mff_path_advance", str),
        ("mff_turn_cmd_topic", "r1_mff_turn_cmd", str),
        ("mff_step_cmd_topic", "r1_mff_step_cmd", str),
        ("mff_status_topic", "r1/task_mff_status", str),
        ("initial_mff_heading_deg", "0", int),
        ("fallback_drive_mode_on_unset", "0", int),
        ("odom_reset_topic", "odom_reset", str),
    ]

    declare_args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default, _ in param_specs
    ]

    node_parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type in param_specs
    }

    node = Node(
        package="r1_planner",
        executable="r1_task_manager",
        name="r1_task_manager",
        output="screen",
        parameters=[node_parameters],
    )

    return LaunchDescription(declare_args + [node])
