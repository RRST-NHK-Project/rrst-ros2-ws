import rclpy
import asyncio
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import flet as ft

class Ros2Publisher(Node):
    def __init__(self):
        super().__init__('flet_ros2_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'slider_values', 10)
        self.msg = Int32MultiArray()

    def publish_data(self, data):
        self.msg.data = data
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Published: {data}')

async def ros_spin(ros_node):
    rclpy.spin_once(ros_node)
    await asyncio.sleep(0.1)  # 少し待機してから再度spinを呼び出す

async def run_ros_and_ui(ros_node, page):
    # 非同期にROS 2ノードの処理とUI更新を並行して実行
    while True:
        # ROS 2のノードを非同期で実行
        await asyncio.gather(
            ros_spin(ros_node),
            page.update_async()  # fletのUIを更新
        )

def main(page: ft.Page):
    rclpy.init()
    ros_node = Ros2Publisher()
    
    # 初期値
    slider_values = [50, 50, 50, 50]
    button_values = [0, 0]

    def on_slider_change(e):
        # スライダーの値を更新し、ROS 2にpublish
        slider_values = [slider1.value, slider2.value, slider3.value, slider4.value]
        ros_node.publish_data(slider_values + button_values)

    def on_button_change(e):
        # ボタンの状態を更新し、ROS 2にpublish
        button_values[0] = button1.value
        button_values[1] = button2.value
        ros_node.publish_data(slider_values + button_values)

    # UIの構成
    slider1 = ft.Slider(min=0, max=100, value=50, on_change=on_slider_change)
    slider2 = ft.Slider(min=0, max=100, value=50, on_change=on_slider_change)
    slider3 = ft.Slider(min=0, max=100, value=50, on_change=on_slider_change)
    slider4 = ft.Slider(min=0, max=100, value=50, on_change=on_slider_change)
    
    button1 = ft.ToggleButton(text="Button 1", value=False, on_change=on_button_change)
    button2 = ft.ToggleButton(text="Button 2", value=False, on_change=on_button_change)
    
    page.add(slider1, slider2, slider3, slider4, button1, button2)

    # 非同期でROS 2のノードとUIを動かす
    asyncio.create_task(run_ros_and_ui(ros_node, page))  # 非同期でタスクを開始

if __name__ == "__main__":
    # flet.appでmain関数を呼び出す際に問題があれば、再確認する
    ft.app(target=main)
