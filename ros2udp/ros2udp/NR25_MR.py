#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
汎用機の機構制御
"""

# パラメーター調整モード、GUIでのパラメーター変更を有効化
GUI_PARAM_MODE = True

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math

# サブモジュール（クラス）のインポート
from .submodules.UDP16 import UDP16
DST_IP = "192.168.8.220"  # 宛先IP
DST_PORT = 5000  # 宛先ポート番号
udp = UDP16(DST_IP, DST_PORT)  # インスタンスを生成

# 以下pipでのインストールが必要
try:
    import pyfiglet
except ModuleNotFoundError:
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("Please install 'pyfiglet' with pip: pip install pyfiglet")
    print(
        "Then, if you have a error 'externally-managed-environment', try: pip install pyfiglet --break-system-packages"
    )
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    exit(1)

data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 

"""
マイコンに送信される配列'data'は17個の要素を持っています。各要素の詳細をここにまとめます。
| data[n] | 詳細 |
| ---- | ---- |
| data[0] | 未使用、送信もされないので注意 |
| data[1] | MD1 |
| data[2] | MD2 |
| data[3] | MD3 |
| data[4] | MD4 |
| data[5] | MD5 |
| data[6] | MD6 |
| data[7] | サーボ1 |
| data[8] | サーボ2 |
| data[9] | サーボ3 |
| data[10] | サーボ4|
| data[11] | 電磁弁1 |
| data[12] | 電磁弁2 |
| data[13] | 電磁弁3 |
| data[14] | 電磁弁4 |
| data[15] | その他通信 |
| data[16] | その他通信 |
"""

duty_max = 90
sp_yaw = 0.1

deadzone = 0.3  # Adjust DS4 deadzone
ready_for_shoot = False

roller_speed_dribble_ab = 30
roller_speed_dribble_cd = 30
roller_speed_shoot_ab = 50
roller_speed_shoot_cd = 50
roller_speed_reload = 10
shoot = 0
dribble = 0


class Listener(Node):

    def __init__(self):
        super().__init__("nhk25_mr")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        print(pyfiglet.figlet_format("NHK2025 MR"))
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

        global ready_for_shoot

        # PSボタンで緊急停止
        if PS:
            print(pyfiglet.figlet_format("HALT"))
            data[1:17] = [0] * 8
            udp.send(data)  # 関数実行
            time.sleep(1)
            while True:
                pass

        # 射出準備
        if CIRCLE:
            Action.ready_for_shoot(self)
            CIRCLE = False
            time.sleep(3)

        # 射出シーケンス
        if ready_for_shoot:
            Action.shoot(self)

        # ドリブル
        if TRIANGLE and not ready_for_shoot:
            Action.dribble(self)

        udp.send(data)  # UDPで送信


class Param_Listener(Node):

    def __init__(self):
        super().__init__("param_listener")
        self.subscription = self.create_subscription(
            Int32MultiArray, "parameter_array", self.param_listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def param_listener_callback(self, gui_msg):

        global roller_speed_dribble_ab
        global roller_speed_dribble_cd
        global roller_speed_shoot_ab
        global roller_speed_shoot_cd
        global shoot
        global dribble

        roller_speed_dribble_ab = gui_msg.data[0]
        roller_speed_dribble_cd = gui_msg.data[1]
        roller_speed_shoot_ab = gui_msg.data[2]
        roller_speed_shoot_cd = gui_msg.data[3]
        shoot = gui_msg.data[4]
        dribble = gui_msg.data[5]

        global ready_for_shoot

        # 射出準備
        if shoot and not ready_for_shoot:
            Action.ready_for_shoot(self)
            shoot = 0
            time.sleep(0.5)

        # 射出シーケンス
        if shoot and ready_for_shoot:
            Action.shoot(self)

        # ドリブル
        if dribble and not ready_for_shoot:
            Action.dribble(self)


class Action:  # 機構制御関数を格納するクラス

    # 射出準備
    def ready_for_shoot(self):
        global ready_for_shoot
        print("Ready for Shooting...")
        data[11] = 1
        data[13] = 1
        data[1] = roller_speed_reload
        data[2] = roller_speed_reload
        data[3] = -1 * roller_speed_reload
        data[4] = -1 * roller_speed_reload
        udp.send(data)  # UDPで送信
        time.sleep(1)
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send(data)  # UDPで送信
        time.sleep(0.5)
        data[1] = -1 * roller_speed_shoot_ab
        data[2] = -1 * roller_speed_shoot_ab
        data[3] = roller_speed_shoot_cd
        data[4] = roller_speed_shoot_cd
        udp.send(data)  # UDPで送信
        ready_for_shoot = True
        print("Done.")

    # 射出シーケンス
    def shoot(self):
        global ready_for_shoot
        print("Shooting...")
        data[12] = 1
        udp.send(data)  # 　UDPで送信
        ready_for_shoot = False
        time.sleep(1.0)
        print("Ready for Retraction...")
        data[12] = 0
        udp.send(data)  # 　UDPで送信
        time.sleep(1.0)
        print("Retracting....")
        data[11] = 0
        data[13] = 0
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send(data)  # UDPで送信
        print("Done.")

    # ドリブル
    def dribble(self):
        print("Dribbling...")
        data[1] = roller_speed_dribble_ab
        data[2] = roller_speed_dribble_ab
        data[3] = -1 * roller_speed_dribble_cd
        data[4] = -1 * roller_speed_dribble_cd
        time.sleep(2.0)
        data[13] = 1
        udp.send(data)  # UDPで送信
        time.sleep(3.0)
        data[13] = 0
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send(data)  # UDPで送信
        print("Done.")


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    listener = Listener()

    if GUI_PARAM_MODE:
        param_listener = Param_Listener()

    exec.add_node(listener)
    if GUI_PARAM_MODE:
        exec.add_node(param_listener)

    exec.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    if GUI_PARAM_MODE:
        param_listener.destroy_node()
    exec.shutdown()


if __name__ == "__main__":
    main()
