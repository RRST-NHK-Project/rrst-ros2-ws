from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="r2_planner",
                executable="r2_task_manager",
                name="r2_task_manager",
                output="screen",
                parameters=[
                    {
                        "initial_state_code": 0,
                        "initial_color_code": -1,
                        "initial_mff_cell": 0,
                    }
                ],
            )
        ]
    )
