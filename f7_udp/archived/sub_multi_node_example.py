#!/usr/bin/env python3
## coding: UTF-8
#:)

"""
ROS2で複数のノードをサブスクライブする例
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math


data = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # モタドラ用のPWMを想定


class Publisher_A(Node):

    def __init__(self):
        super().__init__("publisher_a")
        self.sub = self.create_subscription(
            Int32MultiArray, "node_name", self.pub_a_callback, 10
        )
        self.sub  # prevent unused variable warning

    def pub_a_callback(self, msg):  # YOLOからの入力を処理する関数

        udp.send()  # # UDPで送信する


class Publisher_B(Node):

    def __init__(self):
        super().__init__("publisher_b")
        self.sub = self.create_subscription(
            Int32MultiArray, "node_name", self.pub_a_callback, 10
        )
        self.sub  # prevent unused variable warning

    def pub_a_callback(self, msg):  # YOLOからの入力を処理する関数

        udp.send()  # UDPで送信する


class Publisher_C(Node):

    def __init__(self):
        super().__init__("publisher_c")
        self.sub = self.create_subscription(
            Int32MultiArray, "node_name", self.pub_a_callback, 10
        )
        self.sub  # prevent unused variable warning

    def pub_a_callback(self, msg):  # YOLOからの入力を処理する関数

        udp.send()  # UDPで送信する


class udpsend:
    def __init__(self):

        # SrcIP = "192.168.128.182"  # 送信元IP 家
        # SrcIP = "192.168.2.130"  # 送信元IP 家2
        SrcIP = "192.168.8.195"  # 送信元IP SFT1200
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.215"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)  # 送信元アドレスでバインド

    def send(self):

        # print(data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8])

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

        # print(data[1])

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

    publisher_a = Publisher_A()
    publisher_b = Publisher_B()
    publisher_c = Publisher_C()

    exec.add_node(publisher_a)
    exec.add_node(publisher_b)
    exec.add_node(publisher_c)

    exec.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_a.destroy_node()
    publisher_a.destroy_node()
    publisher_a.destroy_node()
    exec.shutdown()
    # ser.close


if __name__ == "__main__":
    main()
