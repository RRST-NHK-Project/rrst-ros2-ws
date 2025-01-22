#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
４輪オムニのフィードフォワード制御
操舵とアクセル軸を分離
左スティックで操舵、R2でアクセル、右スティックで回転
FBが詰まってるのでFFで操作性の向上を目指す
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from socket import *
import time
import math

# サブモジュール（関数）のインポート
from .submodules.UDP import UDP
DST_IP = "192.168.128.215"  # 宛先IP
DST_PORT = 5000  # 宛先ポート番号
udp = UDP(DST_IP, DST_PORT)  # インスタンスを生成

# 以下pipでのインストールが必要
try:
    import pyfiglet
except ModuleNotFoundError:
    print("Please install 'pyfiglet' with pip: pip install pyfiglet")
    print(
        "Then, if you have a error: externally-managed-environment, try: pip install pyfiglet --break-system-packages"
    )
    exit(1)

data = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # 各モーターの出力（0% ~ 100%）

duty_max = 70
sp_yaw = 0.1

deadzone = 0.3  # adjust DS4 deadzone


class Listener(Node):

    def __init__(self):
        super().__init__("nhk25_mr_omni")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        print(pyfiglet.figlet_format("MR Omni"))
        self.subscription  # prevent unused variable warning

    def listener_callback(self, ps4_msg):
        LS_X = -1 * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[3]
        RS_Y = ps4_msg.axes[4]

        # print(LS_X, LS_Y, RS_X, RS_Y)

        CROSS = ps4_msg.buttons[0]
        CIRCLE = ps4_msg.buttons[1]
        TRIANGLE = ps4_msg.buttons[2]
        SQUARE = ps4_msg.buttons[3]

        LEFT = ps4_msg.axes[6] == 1.0
        RIGHT = ps4_msg.axes[6] == -1.0
        UP = ps4_msg.axes[7] == 1.0
        DOWN = ps4_msg.axes[7] == -1.0

        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]

        L2 = (-1 * ps4_msg.axes[2] + 1) / 2
        R2 = (-1 * ps4_msg.axes[5] + 1) / 2

        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[10]

        L3 = ps4_msg.buttons[11]
        R3 = ps4_msg.buttons[12]

        # PSボタンで緊急停止
        if PS:
            print(pyfiglet.figlet_format("HALT"))
            data[1] = 0
            data[2] = 0
            data[3] = 0
            data[4] = 0
            data[5] = 0
            data[6] = 0
            data[7] = 0
            data[8] = 0
            udp.send(data)  # 関数実行
            time.sleep(1)
            while True:
                pass

        rad = math.atan2(LS_Y, LS_X)
        # vx = math.cos(rad)
        # vy = math.sin(rad)

        v1 = math.sin(rad - 3 * math.pi / 4) * R2
        v2 = math.sin(rad - 5 * math.pi / 4) * R2
        v3 = math.sin(rad - 7 * math.pi / 4) * R2
        v4 = math.sin(rad - 9 * math.pi / 4) * R2

        if RS_X >= deadzone or R1:
            v1 = -1.0 * sp_yaw
            v2 = -1.0 * sp_yaw
            v3 = -1.0 * sp_yaw
            v4 = -1.0 * sp_yaw

        if RS_X <= -1 * deadzone or L1:
            v1 = 1.0 * sp_yaw
            v2 = 1.0 * sp_yaw
            v3 = 1.0 * sp_yaw
            v4 = 1.0 * sp_yaw

        if (
            (math.fabs(LS_X) <= deadzone)
            and (math.fabs(LS_Y) <= deadzone)
            and (math.fabs(RS_X) <= deadzone)
            and (math.fabs(RS_Y) <= deadzone)
            and not R1
            and not L1
        ):
            v1 = 0
            v2 = 0
            v3 = 0
            v4 = 0

        # print(v1, v2, v3, v4)
        data[1] = v1 * duty_max
        data[2] = v2 * duty_max
        data[3] = v3 * duty_max
        data[4] = v4 * duty_max

        udp.send(data)  # 関数実行


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
