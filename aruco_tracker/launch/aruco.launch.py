from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_index',
            default_value='4',
            description='V4L2 カメラデバイスインデックス',
        ),
        DeclareLaunchArgument(
            'marker_length',
            default_value='0.05',
            description='ArUco マーカーの一辺の長さ（メートル）',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='カメラキャプチャのフレームレート',
        ),

        # ArUco 検出・トピック配信ノード
        Node(
            package='aruco_tracker',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'marker_length': LaunchConfiguration('marker_length'),
                'fps': LaunchConfiguration('fps'),
            }],
        ),

        # カメラ映像ビューアノード（オプション：不要ならコメントアウト）
        Node(
            package='aruco_tracker',
            executable='aruco_viewer',
            name='aruco_viewer',
            output='screen',
        ),
    ])
