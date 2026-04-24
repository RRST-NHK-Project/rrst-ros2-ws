import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LD19EightDirectionNode(Node):
    def __init__(self) -> None:
        super().__init__("ld19_eight_direction_node")

        self.declare_parameter("scan_topic", "/ldlidar_node/scan")
        self.declare_parameter("publish_topic", "/ld19/eight_direction_distance")
        self.declare_parameter("output_hz", 2.0)
        self.declare_parameter("window_deg", 2.0)
        self.declare_parameter("front_angle_deg", 0.0)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )
        self.output_hz = max(
            0.1, self.get_parameter("output_hz").get_parameter_value().double_value
        )
        self.window_deg = max(
            0.1, self.get_parameter("window_deg").get_parameter_value().double_value
        )
        self.front_angle_deg = (
            self.get_parameter("front_angle_deg").get_parameter_value().double_value
        )

        self._last_print_sec: float = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, qos)
        self._distance_pub = self.create_publisher(Float32MultiArray, publish_topic, 10)

        self._directions_deg: List[Tuple[str, float]] = [
            ("front", 0.0),
            ("front_left", 45.0),
            ("left", 90.0),
            ("rear_left", 135.0),
            ("rear", 180.0),
            ("rear_right", 225.0),
            ("right", 270.0),
            ("front_right", 315.0),
        ]

        self.get_logger().info(
            f"Listening on {scan_topic}, publishing on {publish_topic}, output {self.output_hz:.1f} Hz"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_print_sec < 1.0 / self.output_hz:
            return
        self._last_print_sec = now_sec

        distances = self._extract_eight_distances(msg)
        self._publish_distances(distances)
        parts = []
        for name, _ in self._directions_deg:
            distance = distances[name]
            if distance is None:
                parts.append(f"{name}:--")
            else:
                parts.append(f"{name}:{distance:.3f}")

        self.get_logger().info("8-dir distance [m] | " + " ".join(parts))

    def _publish_distances(self, distances: Dict[str, Optional[float]]) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float("nan") if distances[name] is None else float(distances[name])
            for name, _ in self._directions_deg
        ]
        self._distance_pub.publish(msg)

    def _extract_eight_distances(self, msg: LaserScan) -> Dict[str, Optional[float]]:
        result: Dict[str, Optional[float]] = {}
        for name, rel_deg in self._directions_deg:
            target_deg = self.front_angle_deg + rel_deg
            result[name] = self._distance_at_angle(
                msg, math.radians(target_deg), math.radians(self.window_deg)
            )
        return result

    @staticmethod
    def _distance_at_angle(
        msg: LaserScan, target_rad: float, window_rad: float
    ) -> Optional[float]:
        if not msg.ranges:
            return None

        best_range: Optional[float] = None
        best_diff = float("inf")

        for i, dist in enumerate(msg.ranges):
            if not math.isfinite(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            diff = abs(
                math.atan2(math.sin(angle - target_rad), math.cos(angle - target_rad))
            )
            if diff > window_rad:
                continue

            if diff < best_diff:
                best_diff = diff
                best_range = dist

        return best_range


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LD19EightDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
