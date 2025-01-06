#!/usr/bin/env python3
## coding: UTF-8

"""
キャチロボ2024
GUIに入力された各パックのワークの数を配列としてをPublishする
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

import cv2
import numpy as np
from ultralytics import YOLO

import flet as ft
import sys


msg = Int32MultiArray()
msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
text = [
    0,
    ft.Text(value="0", size=25, color=ft.colors.RED),
    ft.Text(value="0", size=25, color=ft.colors.GREEN),
    ft.Text(value="0", size=25, color=ft.colors.YELLOW),
    ft.Text(value="0", size=25, color=ft.colors.RED),
    ft.Text(value="0", size=25, color=ft.colors.GREEN),
    ft.Text(value="0", size=25, color=ft.colors.YELLOW),
    ft.Text(value="0", size=25, color=ft.colors.RED),
    ft.Text(value="0", size=25, color=ft.colors.GREEN),
    ft.Text(value="0", size=25, color=ft.colors.YELLOW),
    ft.Text(value="0", size=25, color=ft.colors.RED),
    ft.Text(value="0", size=25, color=ft.colors.GREEN),
    ft.Text(value="0", size=25, color=ft.colors.YELLOW),
    ft.Text(value="0", size=25, color=ft.colors.RED),
    ft.Text(value="0", size=25, color=ft.colors.GREEN),
    ft.Text(value="0", size=25, color=ft.colors.YELLOW),
    ft.Text(value="0", size=25, color=ft.colors.RED),
    ft.Text(value="0", size=25, color=ft.colors.GREEN),
    ft.Text(value="0", size=25, color=ft.colors.YELLOW),
]
mode = False


class cr24_GUI(Node):

    def __init__(self):
        super().__init__("cr24_GUI")
        self.publisher_ = self.create_publisher(Int32MultiArray, "cr24_GUI", 10)
        freq = 0.001  # seconds
        self.timer = self.create_timer(freq, self.timer_callback)
        # self.i = 0

    def timer_callback(self):

        # >>>>>>>>>>>>>>>>>>>>>>Write your code from here>>>>>>>>>>>>>>>>>>>>>>#
        # callbacked every freq[s]

        # -------------------------GUI-------------------------#

        def gui_main(page: ft.Page):
            page.title = "CR24_Control_Panel"  # タイトル
            page.window_width = 300  # 幅
            page.window_height = 600  # 高さ
            page.bgcolor = ft.colors.RED_100
            global text

            # -------------------------GUI Callback-------------------------#

            def mode_change(e):
                global mode
                mode = not mode
                if mode == False:
                    e.page.bgcolor = ft.colors.RED_100
                if mode == True:
                    e.page.bgcolor = ft.colors.BLUE_100
                e.page.update()
                msg.data[0] = mode
                self.publisher_.publish(msg)

            def ems(e):
                pass

            def increase(e):
                i = e.control.data
                msg.data[i] = msg.data[i] + 1
                text[i].value = str(msg.data[i])
                e.page.update()
                self.publisher_.publish(msg)

            def decrease(e):
                i = e.control.data
                msg.data[i] = msg.data[i] - 1
                text[i].value = str(msg.data[i])
                e.page.update()
                self.publisher_.publish(msg)

            # -------------------------GUI Callback-------------------------#

            # -------------------------GUI Object-------------------------#
            page.add(
                ft.Column(
                    [
                        ft.Row(
                            [
                                ft.ElevatedButton("mode", on_click=mode_change),
                                ft.Text("                     "),
                                ft.ElevatedButton(
                                    "EMS", icon="warning_amber", on_click=ems
                                ),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.Text("                     "),
                            ]
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=1, on_click=decrease
                                ),
                                text[1],
                                ft.IconButton(ft.icons.ADD, data=1, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=4, on_click=decrease
                                ),
                                text[4],
                                ft.IconButton(ft.icons.ADD, data=4, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=2, on_click=decrease
                                ),
                                text[2],
                                ft.IconButton(ft.icons.ADD, data=2, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=5, on_click=decrease
                                ),
                                text[5],
                                ft.IconButton(ft.icons.ADD, data=5, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=3, on_click=decrease
                                ),
                                text[3],
                                ft.IconButton(ft.icons.ADD, data=3, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=6, on_click=decrease
                                ),
                                text[6],
                                ft.IconButton(ft.icons.ADD, data=6, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Text("      "),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=7, on_click=decrease
                                ),
                                text[7],
                                ft.IconButton(ft.icons.ADD, data=7, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=10, on_click=decrease
                                ),
                                text[10],
                                ft.IconButton(ft.icons.ADD, data=10, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=8, on_click=decrease
                                ),
                                text[8],
                                ft.IconButton(ft.icons.ADD, data=8, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=11, on_click=decrease
                                ),
                                text[11],
                                ft.IconButton(ft.icons.ADD, data=11, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=9, on_click=decrease
                                ),
                                text[9],
                                ft.IconButton(ft.icons.ADD, data=9, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=12, on_click=decrease
                                ),
                                text[12],
                                ft.IconButton(ft.icons.ADD, data=12, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Text("      "),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=13, on_click=decrease
                                ),
                                text[13],
                                ft.IconButton(ft.icons.ADD, data=13, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=16, on_click=decrease
                                ),
                                text[16],
                                ft.IconButton(ft.icons.ADD, data=16, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=14, on_click=decrease
                                ),
                                text[14],
                                ft.IconButton(ft.icons.ADD, data=14, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=17, on_click=decrease
                                ),
                                text[17],
                                ft.IconButton(ft.icons.ADD, data=17, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                        ft.Row(
                            [
                                ft.IconButton(
                                    ft.icons.REMOVE, data=15, on_click=decrease
                                ),
                                text[15],
                                ft.IconButton(ft.icons.ADD, data=15, on_click=increase),
                                ft.Text("      "),
                                ft.IconButton(
                                    ft.icons.REMOVE, data=18, on_click=decrease
                                ),
                                text[18],
                                ft.IconButton(ft.icons.ADD, data=18, on_click=increase),
                            ],
                            alignment=ft.MainAxisAlignment.CENTER,
                        ),
                    ]
                )
            )
            # -------------------------GUI Object-------------------------#

        # -------------------------GUI-------------------------#

        #ft.app(port=8000, view=ft.WEB_BROWSER, target=gui_main)  # gui_mainをfletで実行(ブラウザ)
        ft.app(target=gui_main)
        # self.get_logger().info('Publishing: "%s"' % msg)

    # >>>>>>>>>>>>>>>>>>>>>>End>>>>>>>>>>>>>>>>>>>>>>#


def main(args=None):
    rclpy.init(args=args)
    CR24_GUI = cr24_GUI()
    rclpy.spin(CR24_GUI)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    CR24_GUI.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
