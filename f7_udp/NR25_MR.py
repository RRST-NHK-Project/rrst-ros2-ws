#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
汎用機の機構制御

2024/12/13: クラス名の変更、機構制御の関数化、GUI連携、大規模な修正のため実機テスト注意
2024/12/16: 実機テスト完了
TODO: 配列dataの拡張
TODO: MD出力のPublish(デバッグ用)
TODO: ds4drvの代替手段の選定
TODO: pyfiglet未インストール時の例外処理
"""

# Falseにすることでルーター未接続でもデバッグ可能、Trueへの戻し忘れに注意
# 追加: アドレスのバインドに失敗すると自動でオフラインモードで開始される機能を実装
ONLINE_MODE = True

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

# 以下pipでのインストールが必要
import pyfiglet

data = [0, 0, 0, 0, 0, 0, -1, -1, -1]  # 各モーターの出力（0% ~ 100%）
# 6, 7, 8, を電磁弁制御に割り当て

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
        print(pyfiglet.figlet_format("MR"))
        self.subscription  # prevent unused variable warning

    def listener_callback(self, ps4_msg):
        LS_X = -1 * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[2]
        RS_Y = ps4_msg.axes[5]

        # print(LS_X, LS_Y, RS_X, RS_Y)

        CROSS = ps4_msg.buttons[1]
        CIRCLE = ps4_msg.buttons[2]
        TRIANGLE = ps4_msg.buttons[3]
        SQUARE = ps4_msg.buttons[0]

        LEFT = ps4_msg.axes[12] == 1.0
        RIGHT = ps4_msg.axes[12] == -1.0
        UP = ps4_msg.axes[13] == 1.0
        DOWN = ps4_msg.axes[13] == -1.0

        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]

        L2 = (-1 * ps4_msg.axes[3] + 1) / 2
        R2 = (-1 * ps4_msg.axes[4] + 1) / 2

        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]

        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]

        global ready_for_shoot

        # PSボタンで緊急停止
        if PS:
            print(pyfiglet.figlet_format("HALT"))
            data[1] = 0
            data[2] = 0
            data[3] = 0
            data[4] = 0
            data[5] = 0
            data[6] = -1
            data[7] = -1
            data[8] = -1
            udp.send()  # UDPで送信
            time.sleep(1)
            while True:
                pass

        # 射出準備
        if CIRCLE and not ready_for_shoot:
            Action.ready_for_shoot()
            CIRCLE = False
            time.sleep(0.5)

        # 射出シーケンス
        if CIRCLE and ready_for_shoot:
            Action.shoot()

        # ドリブル
        if TRIANGLE and not ready_for_shoot:
            Action.dribble()

        udp.send()  # UDPで送信


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
            Action.ready_for_shoot()
            shoot = 0
            time.sleep(0.5)

        # 射出シーケンス
        if shoot and ready_for_shoot:
            Action.shoot()

        # ドリブル
        if dribble and not ready_for_shoot:
            Action.dribble()


class Action:  # 機構制御関数を格納するクラス

    # 射出準備
    def ready_for_shoot():
        global ready_for_shoot
        print("Ready for Shooting...")
        data[6] = 1
        data[8] = 1
        data[1] = roller_speed_reload
        data[2] = roller_speed_reload
        data[3] = -1 * roller_speed_reload
        data[4] = -1 * roller_speed_reload
        udp.send()  # UDPで送信
        time.sleep(1)
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send()  # UDPで送信
        time.sleep(0.5)
        data[1] = -1 * roller_speed_shoot_ab
        data[2] = -1 * roller_speed_shoot_ab
        data[3] = roller_speed_shoot_cd
        data[4] = roller_speed_shoot_cd
        udp.send()  # UDPで送信
        ready_for_shoot = True
        print("Done.")

    # 射出シーケンス
    def shoot():
        global ready_for_shoot
        print("Shooting...")
        data[7] = 1
        udp.send()  # 　UDPで送信
        ready_for_shoot = False
        time.sleep(1.0)
        print("Ready for Retraction...")
        data[7] = -1
        udp.send()  # 　UDPで送信
        time.sleep(1.0)
        print("Retracting....")
        data[6] = -1
        data[8] = -1
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send()  # UDPで送信
        print("Done.")

    # ドリブル
    def dribble():
        print("Dribbling...")
        data[8] = 1
        data[1] = roller_speed_dribble_ab
        data[2] = roller_speed_dribble_ab
        data[3] = -1 * roller_speed_dribble_cd
        data[4] = -1 * roller_speed_dribble_cd
        udp.send()  # UDPで送信
        time.sleep(6.0)
        data[8] = -1
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send()  # UDPで送信
        print("Done.")


class UDP:  # UDP通信のクラス
    def __init__(self):

        SrcIP = "192.168.8.196"  # 送信元IP
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.216"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        try:  # 送信元アドレスでバインド
            self.udpClntSock.bind(self.SrcAddr)
        except:  # 例外処理、バインドに失敗したときはオフラインモードで開始
            print("Cannot assign requested address.\nOFFLINE Mode started.")
            ONLINE_MODE = False

    def send(self):

        # print(data[1], data[2], data[3], data[4])

        # Duty比のリミッター、消すな！
        for i in range(len(data)):
            if data[i] > duty_max:
                data[i] = duty_max
            elif data[i] < -duty_max:
                data[i] = -duty_max

        str_data = (
            str(data[1])
            + ","
            + str(data[2])
            + ","
            + str(data[3])
            + ","
            + str(data[4])
            + ","
            + str(data[5])
            + ","
            + str(data[6])
            + ","
            + str(data[7])
            + ","
            + str(data[8])
        )  # パケットを作成

        print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        if ONLINE_MODE:
            self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        # data[1] = 0
        # data[2] = 0
        # data[3] = 0
        # data[4] = 0
        # data[5] = 0
        # ata[6] = 0
        # data[7] = 0
        # data[8] = 0


udp = UDP()


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
