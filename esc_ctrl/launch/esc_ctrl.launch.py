from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="esc_ctrl",
                executable="esc_ctrl_node",
                name="esc_ctrl",
                output="screen",
                parameters=[
                    {
                        "device_id": 153,
                        "tx_period_ms": 20,
                        "log_period_ms": 200,
                        "fixed_enable": 1,
                        "fixed_mode": 0,
                        "fixed_target": 5.0,
                        "fixed_voltage_limit": 12.0,
                    }
                ],
            )
        ]
    )
