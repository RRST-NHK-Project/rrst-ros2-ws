import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray


data = [0, 0, True]  # グローバル変数


class KFS_judge_node(Node):

    def __init__(self):
        super().__init__("kfs_judge_node")
        self.sub = self.create_subscription(
            Int32MultiArray, "/KFS_judge", self.kfs_judge_callback, 10
        )
        self.sub
        self.publisher_ = self.create_publisher(Int32MultiArray, "KFS_judge", 10)

        self.info_sub = self.create_subscription(
            Float32MultiArray,
            "/cube_detection/info",
            self.info_callback,
            10
        )
        self.latest_info = None
        

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

# call back
def info_callback(self, msg):
    self.latest_info = msg.data # 最新検出情報を変数に保存

    if len(msg.data) < 8 :
        return
    self.target_data = {
        'detected': bool(msg.data[0]),
        'cx': msg.data[1],  # 画像内の中心X (0.0~1.0)
        'cy': msg.data[2],  # 画像内の中心Y (0.0~1.0)
        'depth': msg.data[5] # 距離 (m)
    }    

def image_callback(self, msg):
    if self.latest_info is None or self.latest_info[0] == 0.0:  # 未検出ならスキップ  # 最新情報がない、または検出されていない場合
        return
    
    flag = self
    cx_norn = self.latest_info[1] # 画像内の中心X (0.0~1.0)
    cy_norn = self.latest_info[2] # 画像内の中心Y
    w_norm = self.latest_info[3]  # 画像内の幅 (0.0~1.0)
    h_norm = self.latest_info[4]  # 画像内の高さ (0.0~1.0)
    depth_m = self.latest_info[5] # 深度距離 (m)


def main(args=None):
    rclpy.init(args=args)
    judge_node = KFS_judge_node()

    rclpy.spin(judge_node)

    judge_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
