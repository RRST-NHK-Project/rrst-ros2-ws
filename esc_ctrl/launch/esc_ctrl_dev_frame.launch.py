from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="esc_ctrl",
                executable="esc_ctrl_dev_frame",
                name="esc_ctrl_dev_frame",
                output="screen",
                parameters=[
                    {
                        "device_id": 153,
                        "tx_period_ms": 20,
                        "joy_topic": "joy",
                        "enable_on_start": True,
                        "start_mode": 0,
                        "start_velocity_rpm": 0.0,
                        "start_angle_deg": 0.0,
                        "start_voltage_limit": 12.0,
                        "show_rx_logs": True,
                    }
                ],
            )
        ]
    )