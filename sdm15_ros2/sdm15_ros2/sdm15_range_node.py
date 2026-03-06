import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32

from .SDM15 import SDM15, BaudRate


class SDM15Node(Node):
    def __init__(self):
        super().__init__("sdm15_node")

        # ===== parameters =====
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", BaudRate.BAUD_460800)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        # ===== LiDAR =====
        self.lidar = SDM15(port, BaudRate(baud))

        self.get_logger().info("Obtaining version info...")
        info = self.lidar.obtain_version_info()
        self.get_logger().info(
            f"Model={info.model} HW={info.hardware_version} "
            f"FW={info.firmware_version_major}.{info.firmware_version_minor}"
        )

        self.lidar.lidar_self_test()
        self.get_logger().info("Self test OK")

        self.lidar.start_scan()
        self.get_logger().info("Scan started")

        # ===== shared data =====
        self._lock = threading.Lock()
        self._latest_distance = None
        self._latest_intensity = None
        self._latest_disturb = None
        self._running = True

        # ===== LiDAR thread =====
        self._lidar_thread = threading.Thread(target=self._lidar_loop, daemon=True)
        self._lidar_thread.start()

        # ===== publisher =====
        self.pub = self.create_publisher(
            Int32, "sdm15/distance", qos_profile_sensor_data
        )

        # ===== timer (publish only) =====
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

    # ===============================
    # LiDAR communication thread
    # ===============================
    def _lidar_loop(self):
        self.get_logger().info("SDM15 thread started")
        while self._running:
            try:
                distance, intensity, disturb = self.lidar.get_distance()
                with self._lock:
                    self._latest_distance = distance
                    self._latest_intensity = intensity
                    self._latest_disturb = disturb
            except Exception as e:
                self.get_logger().warn_throttle(5.0, f"LiDAR error: {e}")
                time.sleep(0.01)

    # ===============================
    # ROS 2 timer callback
    # ===============================
    def timer_callback(self):
        with self._lock:
            if self._latest_distance is None:
                return
            distance = self._latest_distance

        msg = Int32()
        msg.data = distance
        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Stopping SDM15...")
        self._running = False
        try:
            self.lidar.stop_scan()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SDM15Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
