import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class IMUToOdometry(Node):
    def __init__(self):
        super().__init__("imu_to_odometry")

        qos_profile = QoSProfile(  # imuのノードがROS1から移植されたものなのでQoSプロファイルを合わせる必要がある
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile)
        self.publisher = self.create_publisher(Odometry, "/odom", 10)

        # 初期化
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_time = self.get_clock().now().to_msg()

        self.get_logger().info("IMU to Odometry Node has been started.")

    def imu_callback(self, msg):
        current_time = self.get_clock().now().to_msg()
        dt = (current_time.sec + current_time.nanosec * 1e-9) - (
            self.last_time.sec + self.last_time.nanosec * 1e-9
        )
        self.last_time = current_time

        # 加速度データを取得
        acceleration = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )

        # 速度を更新
        self.velocity += acceleration * dt

        # 位置を更新
        self.position += self.velocity * dt

        # Odometryメッセージを作成
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]

        # 現在の速度を設定
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        # Odometryメッセージをパブリッシュ
        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_to_odometry = IMUToOdometry()
    rclpy.spin(imu_to_odometry)
    imu_to_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
