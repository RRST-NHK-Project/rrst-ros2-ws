from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="sdm15_ros2",
            executable="sdm15_range_node",
            name="sdm15_node_1",
            namespace="lidar1",
            parameters=[
                {"port": "/dev/ttyUSB0"},
                {"baud": 460800},
            ],
            output="screen",
        ),

        Node(
            package="sdm15_ros2",
            executable="sdm15_range_node",
            name="sdm15_node_2",
            namespace="lidar2",
            parameters=[
                {"port": "/dev/ttyUSB1"},
                {"baud": 460800},
            ],
            output="screen",
        ),

    ])