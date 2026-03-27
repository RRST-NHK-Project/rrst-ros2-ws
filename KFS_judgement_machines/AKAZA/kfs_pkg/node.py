# 画像認識のxy座標と真偽だけ送るノード

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math

data = [0, 0, True]  # グローバル変数


class KFS_judge_node(Node):

    def __init__(self):
        super().__init__("kfs_judge_node")
        self.sub = self.create_subscription(
            Int32MultiArray, "/KFS_judge", self.kfs_judge_callback, 10
        )
        self.sub
        self.publisher_ = self.create_publisher(Int32MultiArray, "KFS_judge", 10)

    def kfs_judge_callback(self, msg):
        global data
        # データが正しく3つ以上入っているかチェックする
        if len(msg.data) >= 3:
            data[0] = msg.data[0]
            data[1] = msg.data[1]
            data[2] = bool(msg.data[2])  # 数値を真偽値に変換

            self.get_logger().info(f"UDP送信準備: {data}")
            # クラス外の udp インスタンスの send メソッドを呼ぶ
            udp.send()
        else:
            self.get_logger().warn(f"受信データの要素が足りません: {msg.data}")


class udpsend:
    def __init__(self):
        SrcIP = ""  # 送信元IP SFT1200
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.215"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)  # 送信元アドレスでバインド

    def send(self):

        kfs_data = (
            str(data[0]) + "," + str(data[1]) + "," + str(data[2])
        )  # パケットを作成

        send_data = kfs_data.encode("utf-8")  # 文字列をバイト型に変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        data[0] = 0
        data[1] = 0
        data[2] = True


udp = udpsend()


def main(args=None):
    rclpy.init(args=args)
    judge_node = KFS_judge_node()

    rclpy.spin(judge_node)

    judge_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
