# 必要なpythonモジュールをインポート
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
# 以下に起動に関する詳細を記述
def generate_launch_description():
   return LaunchDescription([
    Node(
                package="joy",
                executable="joy_node",
                output="screen",
                parameters=[{"device_id": 0}],  # ここで device_id を指定
                remappings=[("/joy", "/joy0")],
            ),
            Node(
                package="cr25tashiron",
                executable="cr25_matsu",
                output="screen",
            ),
            ExecuteProcess(
            cmd=[
                "sudo", "docker", "run", "--rm",
                "-v", "/dev:/dev",
                "--privileged",
                "--net=host",
                "microros/micro-ros-agent:jazzy",
                "serial",
                "--dev", "/dev/ttyUSB0",
                "-baudrate", "1000000",
                "-v6"
            ],
            output="screen"
        ),
          
   ])

