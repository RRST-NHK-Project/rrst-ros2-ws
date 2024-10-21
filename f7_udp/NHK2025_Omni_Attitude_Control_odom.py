#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
4輪オムニホイールロボットの制御プログラム
ジョイスティック入力に基づいてロボットの動きを制御し、UDPで速度指令を送信する
2024/10/21
"""

# オンラインモードの設定（ルーター接続時はTrue、デバッグ時はFalse）
online_mode = True

# 必要なライブラリのインポート
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from socket import *
import time
import math
import pyfiglet

# モーター速度データの初期化 (0%-100%)
data = [0, 0, 0, 0, 0, 0, 0, 0, 0]

# オムニホイールパラメーター
fth = 0  # 前方向の閾値
vth = 0  # 速度閾値
r = 0  # 回転半径
rpm_limit = 15  # RPM制限
sp_yaw = 0.5  # ヨー回転速度
sp_omni = 1.0  # オムニ移動速度

# ジョイスティックのデッドゾーン
deadzone = 0.3


class Listener(Node):
    def __init__(self):
        super().__init__("nhk25_omni_driver")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        # プログラム開始メッセージの表示
        print(pyfiglet.figlet_format("NHK2025"))
        if not online_mode:
            print("[OFFLINE] Omni driver started.\nlimit{}")

    def listener_callback(self, ps4_msg):
        # ジョイスティックの入力値を取得
        LS_X = -1 * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[2]
        RS_Y = ps4_msg.axes[5]

        # ボタン入力の取得
        CROSS = ps4_msg.buttons[1]
        CIRCLE = ps4_msg.buttons[2]
        TRIANGLE = ps4_msg.buttons[3]
        SQUARE = ps4_msg.buttons[0]

        # 方向キーの入力取得
        LEFT = ps4_msg.axes[12] == 1.0
        RIGHT = ps4_msg.axes[12] == -1.0
        UP = ps4_msg.axes[13] == 1.0
        DOWN = ps4_msg.axes[13] == -1.0

        # その他のボタン入力
        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]
        L2 = ps4_msg.buttons[6]
        R2 = ps4_msg.buttons[7]
        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]
        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]

        # 方向キー入力の処理
        if UP == 1:
            LS_Y = 1.0
        if DOWN == 1:
            LS_Y = -1.0
        if LEFT == 1:
            LS_X = -1.0
        if RIGHT == 1:
            LS_X = 1.0

        # 移動方向の計算
        rad = math.atan2(LS_Y, LS_X)

        # 各ホイールの速度計算
        v1 = math.sin(rad - math.pi / 4) * sp_omni
        v2 = math.sin(rad - 3 * math.pi / 4) * sp_omni
        v3 = math.sin(rad - 5 * math.pi / 4) * sp_omni
        v4 = math.sin(rad - 7 * math.pi / 4) * sp_omni

        # 右スティックによる回転
        if RS_X >= deadzone:
            R2 = 1
        if RS_X <= -1 * deadzone:
            L2 = 1

        # 回転動作
        if R2 == 1:
            v1 = v2 = v3 = v4 = -1.0 * sp_yaw
        if L2 == 1:
            v1 = v2 = v3 = v4 = 1.0 * sp_yaw

        # 停止条件
        if (
            (math.fabs(LS_X) <= deadzone)
            and (math.fabs(LS_Y) <= deadzone)
            and R2 == 0
            and L2 == 0
        ):
            v1 = v2 = v3 = v4 = 0

        # 速度データの設定
        data[1] = v1 * (rpm_limit + 1)
        data[2] = v2 * (rpm_limit + 1)
        data[3] = v3 * (rpm_limit + 1)
        data[4] = v4 * (rpm_limit + 1)

        # オンラインモードの場合、UDPで送信
        if online_mode:
            udp.send()


class udpsend:
    def __init__(self):
        # UDP通信の設定
        SrcIP = "192.168.8.195"  # 送信元IP
        SrcPort = 4000  # 送信元ポート
        self.SrcAddr = (SrcIP, SrcPort)

        DstIP = "192.168.8.215"  # 宛先IP
        DstPort = 5000  # 宛先ポート
        self.DstAddr = (DstIP, DstPort)

        # UDPソケットの作成とバインド
        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)
        self.udpClntSock.bind(self.SrcAddr)

        print(f"Omni driver started.\nSrcIP: {SrcIP}\nDstIP: {DstIP}")

    def send(self):

        # print(data[1], data[2], data[3], data[4])

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

        #print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        data[5] = 0
        data[6] = 0
        data[7] = 0
        data[8] = 0


# オンラインモード時のUDPインスタンス作成
if online_mode:
    udp = udpsend()


def main(args=None):
    # ROSの初期化
    rclpy.init(args=args)

    # Listenerノードの作成と実行
    listener = Listener()
    rclpy.spin(listener)

    # ノードの終了処理
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
