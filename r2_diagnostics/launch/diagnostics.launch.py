from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for diagnostics node"""

    # Diagnostics node
    diagnostics_node = Node(
        package="r2_diagnostics",
        executable="diagnostics_node",
        name="r2_diagnostics",
        output="screen",
        parameters=[],
        remappings=[
            ("serial_rx_7", "/serial_rx_7"),
            ("joy", "/joy"),
        ],
    )

    return LaunchDescription(
        [
            diagnostics_node,
        ]
    )
