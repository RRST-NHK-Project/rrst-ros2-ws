#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
サーボ独ステの制御！
"""

# Falseにすることでルーター未接続でもデバッグ可能、Trueへの戻し忘れに注意
# アドレスのバインドに失敗すると自動でオフラインモードで開始される
ONLINE_MODE = True

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from socket import *
import time
import math

# 以下pipでのインストールが必要
import pyfiglet

data = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # 各モーターの出力（0% ~ 100%）

duty_max = 70
sp_yaw = 0.1
wheelspeed = 20

deadzone = 0.3  # adjust DS4 deadzone


class Listener(Node):

    def __init__(self):
        super().__init__("nhk25_mr_sd")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        print(pyfiglet.figlet_format("MR SwerveDrive"))
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
            udp.send()  # 関数実行
            time.sleep(1)
            while True:
                pass

        rad = math.atan2(LS_Y, LS_X)
        if math.fabs(LS_X)  > deadzone or math.fabs(LS_Y) > deadzone:
            deg = int(rad * 180 / math.pi)
            if deg < 0:
                deg += 360
            print(deg)
            data[1] = deg
        
        """
        v1 = wheelspeed * R2
        v2 = wheelspeed * R2
        v3 = wheelspeed * R2
        v4 = wheelspeed * R2

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
        """
        udp.send()  # 関数実行


class udpsend:
    def __init__(self):

        try:
            # ダミー接続を使ってIPアドレスを取得
            with socket(AF_INET, SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))  # Google DNSに接続 (実際には接続しない)
                ip_address = s.getsockname()[0]
        except Exception as e:
            return f"Getting IP Error: {e}"

        SrcIP = ip_address  # 送信元IP
        SrcPort = 0  # 送信元ポート番号,0にすることでポートが自動割り当てされる。これにより複数ノードでポートを使い分けることができる。
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.215"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        try:  # 送信元アドレスでバインド
            self.udpClntSock.bind(self.SrcAddr)
        except:  # 例外処理、バインドに失敗したときはオフラインモードで開始
            print("Cannot assign requested address.\nOFFLINE Mode started.")
            ONLINE_MODE = False

    def send(self):

        if ONLINE_MODE:
            # print(data[1], data[2], data[3], data[4])
            """
            # Duty比のリミッター、消すな！
            for i in range(len(data)):
                if data[i] > duty_max:
                    data[i] = duty_max
                elif data[i] < -duty_max:
                    data[i] = -duty_max
            """
            str_data = ','.join(map(str, data[1:9]))  # カンマ区切りで文字列化

            # print(str_data)

            send_data = str_data.encode("utf-8")  # バイナリに変換

            self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信


udp = udpsend()  # クラス呼び出し


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
