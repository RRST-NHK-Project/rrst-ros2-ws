import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math
import time

class OmniWheelOdometry(Node):
    def __init__(self):
        super().__init__('omni_wheel_odometry')
        
        # パラメータ設定
        self.encoder_resolution = 1000  # エンコーダー1回転あたりのパルス数
        self.wheel_diameter = 0.1       # 車輪の直径（メートル）
        self.wheel_base = 0.5           # 車輪間の幅（メートル）
        self.last_time = self.get_clock().now()

        # エンコーダーパルス
        self.encoder_left = 0
        self.encoder_right = 0

        # 初期位置と向き
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # /odomトピックのパブリッシャー
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # エンコーダー値の購読者
        self.create_subscription(
            Odometry, '/encoder_left', self.encoder_left_callback, 10)
        self.create_subscription(
            Odometry, '/encoder_right', self.encoder_right_callback, 10)

        # タイマーで周期的に計算
        self.create_timer(0.1, self.update_odometry)

    def encoder_left_callback(self, msg):
        self.encoder_left = msg.data

    def encoder_right_callback(self, msg):
        self.encoder_right = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # エンコーダーから進んだ距離を計算
        left_distance = (self.encoder_left / self.encoder_resolution) * math.pi * self.wheel_diameter
        right_distance = (self.encoder_right / self.encoder_resolution) * math.pi * self.wheel_diameter

        # 台車の直進と回転速度を計算
        delta_s = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # ロボットの姿勢を更新
        self.theta += delta_theta
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)

        # Odometryメッセージの作成
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # 位置と向き
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # 速度
        odom_msg.twist.twist.linear.x = delta_s / delta_time
        odom_msg.twist.twist.angular.z = delta_theta / delta_time

        # 公開
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelOdometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
