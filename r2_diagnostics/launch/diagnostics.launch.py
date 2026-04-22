from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for diagnostics node"""

    # Diagnostics node
    # startup_safety_device_ids: [] means auto-discover all existing serial_tx_* / serial_rx_* IDs.
    diagnostics_node = Node(
        package="r2_diagnostics",
        executable="diagnostics_node",
        name="r2_diagnostics",
        output="screen",
        parameters=[
            {
                "startup_safety_enabled": True,
                "startup_safety_window_sec": 8.0,
                "startup_safety_exit_after_check": True,
            }
        ],
        remappings=[
            ("joy", "/joy"),
        ],
    )

    return LaunchDescription(
        [
            diagnostics_node,
        ]
    )
