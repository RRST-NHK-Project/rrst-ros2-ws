#画像認識のxy座標と真偽だけ送るノード

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math

data = [0,0,True]#グローバル変数

class KFS_judge_node(Node):

    def __init__(self):
        super().__init__('kfs_judge_node')
        self.sub = self.create_subscription(
            Int32MultiArray, "KFS_judge", self.kfs_judge_callback, 10
        )
        self.sub
        self.publisher_ = self.create_publisher(Int32MultiArray, 'KFS_judge', 10)

    def kfs_judge_callback(self, msg):
        global data
        data[0] = msg.data[0]
        data[1] = msg.data[1]
        data[2] = msg.data[2]

        self.get_logger().info(f'UDP送信準備: {data}')
        udp.send()

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

        kfs_data = (
            str(data[0])
            + ","
            + str(data[1])
            + ","
            + str(data[2])
        )   #パケットを作成

        send_data = kfs_data.encode("utf-8")  # 文字列をバイト型に変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        data[0] = 0
        data[1] = 0
        data[2] = True

udp = udpsend()

def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    kfs_judge_node = KFS_judge_node()

    exec.add_node(kfs_judge_node)

    exec.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kfs_judge_node.destroy_node()
    exec.shutdown()
    # ser.close


if __name__ == "__main__":
    main()