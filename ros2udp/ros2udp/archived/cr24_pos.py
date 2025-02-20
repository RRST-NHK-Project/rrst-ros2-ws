#!/usr/bin/env python3
## coding: UTF-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from socket import *


class setoshio_pub(Node):

    def __init__(self):
        super().__init__("pos")
        self.publisher_ = self.create_publisher(Int32MultiArray, "cr24_pos", 10)
        freq = 0.01  # seconds
        self.timer = self.create_timer(freq, self.timer_callback)
        # self.i = 0

    def timer_callback(self):  # callback for publishing setoshio data

        # >>>>>>>>>>>>>>>>>>>>>>Write your code from here>>>>>>>>>>>>>>>>>>>>>>#
        # callbacked every freq[s]

        udp.recv()

        # >>>>>>>>>>>>>>>>>>>>>>End>>>>>>>>>>>>>>>>>>>>>>#


class udprecv:
    def __init__(self):

        SrcIP = "192.168.8.197"  # 受信元IP
        SrcPort = 4000  # 受信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        self.BUFSIZE = 1024  # バッファサイズ指定
        self.udpServSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpServSock.bind(self.SrcAddr)  # 受信元アドレスでバインド

    def recv(self):

        data, addr = self.udpServSock.recvfrom(self.BUFSIZE)
        # 受信
        print(data.decode(), addr)  # 受信データと送信アドレス表示


udp = udprecv()  # クラス呼び出し


def main(args=None):
    rclpy.init(args=args)
    setoshio_publisher = setoshio_pub()
    rclpy.spin(setoshio_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    setoshio_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
