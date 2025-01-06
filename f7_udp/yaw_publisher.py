#GPTに書かせたけどうまく動かなかった

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class YawPublisher(Node):
    def __init__(self):
        super().__init__("yaw_publisher")

        qos_profile = QoSProfile(  # imuのノードがROS1から移植されたものなのでQoSプロファイルを合わせる必要がある
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            Imu, "/imu", self.listener_callback, qos_profile
        )
        self.publisher = self.create_publisher(Float64, "/yaw", 10)
        self.get_logger().info("Yaw Publisher is running")

    def listener_callback(self, msg):
        # IMUのクォータニオンからYAWを計算
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # YAW角を計算（ラジアン）
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        # ラジアンを度に変換
        yaw_degrees = math.degrees(yaw)

        # YAWをPublish
        yaw_msg = Float64()
        yaw_msg.data = yaw_degrees
        self.publisher.publish(yaw_msg)


def main(args=None):
    rclpy.init(args=args)
    yaw_publisher = YawPublisher()
    rclpy.spin(yaw_publisher)
    yaw_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
