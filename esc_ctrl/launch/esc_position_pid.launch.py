from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="esc_ctrl",
                executable="esc_position_pid_node",
                name="esc_position_pid",
                output="screen",
                parameters=[
                    {
                        "device_id": 0,
                        "rx_device_id": 0,
                        "tx_period_ms": 5,
                        "command_topic_prefix": "esc_cmd_pos",
                        "command_rpm_scale": 1.0,
                        "voltage_limit": 12.0,
                        "position_tolerance_deg": 1.0,
                        "enable_on_command": True,
                        "show_info_logs": True,
                        "pid_kp": 10.0,
                        "pid_ki": 0.0,
                        "pid_kd": 0.0,
                        "pid_max_rpm": 3000.0,
                    }
                ],
            )
        ]
    )