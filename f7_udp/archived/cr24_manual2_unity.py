#!/usr/bin/env python3
## coding: UTF-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math

data = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # モタドラ用のPWMを想定
fth = 0
vth = 0
r = 0
rpm_limit = 120
sp_yaw = 0.5
sp_omni = 1.0

deadzone = 0.3  # adjust DS4 deadzone


class Listener(Node):

    def __init__(self):
        super().__init__("ps4_manual_cr24_2")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, ps4_msg):
        LS_X = ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        # RS_X = (-1) * ps4_msg.axes[2]
        # RS_Y = ps4_msg.axes[5]

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

        L2 = ps4_msg.buttons[6]
        R2 = ps4_msg.buttons[7]

        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]

        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]

        rad = math.atan2(LS_Y, LS_X)

        if UP:
            data[7] = -30  # 上押したらハンド軸前進
        if DOWN:
            data[7] = 30  # 下押したらハンド軸後退

        if LEFT:
            data[5] = 30
        if RIGHT:
            data[5] = -30

        if TRIANGLE:
            data[1] = 30
        if CROSS:
            data[1] = -30

        if CIRCLE:
            data[3] = 1
        if SQUARE:
            data[3] = 0

        if L1:
            data[2] = -30
        if L2:
            data[2] = 30

        if R2:
            if UP:
                data[7] = -60  # 上押したらハンド軸前進
            if DOWN:
                data[7] = 60  # 下押したらハンド軸後退

            if LEFT:
                data[5] = 60
            if RIGHT:
                data[5] = -60

            if TRIANGLE:
                data[1] = 60
            if CROSS:
                data[1] = -60

            if CIRCLE:
                data[3] = 1
            if SQUARE:
                data[3] = 0

            if L1:
                data[2] = -60
            if L2:
                data[2] = 60

        """
        UP:ハンド前進
        DOWN:ハンド後退
        LEFT:左移動
        RIGHT:右移動
        CROSS:ハンド軸後退
        CIRCLE: 
        TRIANGLE: ハンド軸前進
        SQUARE: 
        L1:上昇
        L2:下降
        R1:
        R2:押しっぱで全部速くなる
        """
        # print("ok")
        udp.send()  # 関数実行


class Unity_Listener(Node):

    def __init__(self):
        super().__init__("Unity_handler")
        self.sub3 = self.create_subscription(
            Int32MultiArray, "hello_from_Unity", self.unity_callback, 10
        )
        self.sub3  # prevent unused variable warning

    def unity_callback(self, unity_msg):

        BUTTON1 = unity_msg.data[1] == 1
        BUTTON2 = unity_msg.data[2] == 1
        BUTTON3 = unity_msg.data[3] == 1
        BUTTON4 = unity_msg.data[4] == 1
        BUTTON5 = unity_msg.data[5] == 1
        BUTTON6 = unity_msg.data[6] == 1
        BUTTON7 = unity_msg.data[7] == 1
        BUTTON8 = unity_msg.data[8] == 1
        BUTTON9 = unity_msg.data[9] == 1
        BUTTON10 = unity_msg.data[10] == 1

        if BUTTON5:
            data[7] = -80  # 上押したらハンド軸前進
        if BUTTON2:
            data[7] = 80  # 下押したらハンド軸後退

        if BUTTON8:
            data[5] = 60
        if BUTTON7:
            data[5] = -60

        if BUTTON6:
            data[1] = 80
        if BUTTON3:
            data[1] = -80

        if BUTTON9:
            data[3] = 1

        if BUTTON10:
            data[3] = 2

        if BUTTON4:
            data[2] = -70
        if BUTTON1:
            data[2] = 70

        udp.send()  # 関数実行


class udpsend:
    def __init__(self):

        SrcIP = "192.168.8.196"  # 送信元IP
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.216"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)  # 送信元アドレスでバインド

    def send(self):

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
        # str_data = (str(data[1])+str(data[2])+str(data[3])+str(data[4])+str(data[5]))
        send_data = str_data.encode("utf-8")  # バイナリに変換
        # binary = data.to_bytes(4,'big')

        print(data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8])

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        data[5] = 0
        data[6] = 0
        data[7] = 0
        data[8] = 0


udp = udpsend()  # クラス呼び出し


def main(args=None):
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()

    listener = Listener()
    unity_listener = Unity_Listener()

    exec.add_node(listener)
    exec.add_node(unity_listener)

    exec.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    unity_listener.destroy_node()
    exec.shutdown()
