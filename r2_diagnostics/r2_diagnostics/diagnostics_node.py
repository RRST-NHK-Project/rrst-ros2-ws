"""
ROS2 Diagnostics Node for R2 robot
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Int32MultiArray, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger

import time
import json
import re
from pathlib import Path

from .hardware_diagnostics import HardwareDiagnostics, DiagnosticStatus as HDStatus
from .sensor_diagnostics import SensorDiagnostics, SensorType
from .utils import (
    print_colored_text,
    create_diagnostics_report,
    save_diagnostics_report,
)


class R2DiagnosticsNode(Node):
    """ROS2 node for R2 robot self-diagnostics"""

    def __init__(self):
        super().__init__("r2_diagnostics_node")

        # Parameters for startup safety check
        self.declare_parameter("startup_safety_enabled", True)
        self.declare_parameter("startup_safety_device_ids", [])
        self.declare_parameter("startup_safety_window_sec", 8.0)
        self.declare_parameter("startup_safety_exit_after_check", True)

        # Initialize diagnostics
        self.hw_diagnostics = HardwareDiagnostics()
        self.sensor_diagnostics = SensorDiagnostics()

        # Create callback group for threading
        callback_group = ReentrantCallbackGroup()
        self.callback_group = callback_group

        # Startup safety state
        self.startup_safety_enabled = bool(
            self.get_parameter("startup_safety_enabled").value
        )
        self.startup_safety_device_ids = [
            int(v) for v in self.get_parameter("startup_safety_device_ids").value
        ]
        self.startup_safety_auto_discover = len(self.startup_safety_device_ids) == 0
        self.startup_safety_window_sec = float(
            self.get_parameter("startup_safety_window_sec").value
        )
        self.startup_safety_exit_after_check = bool(
            self.get_parameter("startup_safety_exit_after_check").value
        )
        self.startup_safety_start_time = time.time()
        self.startup_safety_completed = not self.startup_safety_enabled
        self.startup_safety_seen_topics = set()
        self.startup_safety_warned_topics = set()
        self.startup_safety_issue_details = {}
        self.startup_safety_subs = {}
        self.startup_safety_monitored_topics = set()
        self.startup_safety_monitored_device_ids = set()
        self.shutdown_timer = None

        self.startup_safety_diag_level = DiagnosticStatus.OK
        self.startup_safety_diag_message = "disabled"
        if self.startup_safety_enabled:
            self.startup_safety_diag_level = DiagnosticStatus.WARN
            self.startup_safety_diag_message = "startup serial safety check in progress"

        # Publishers
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.diagnostics_text_pub = self.create_publisher(
            String, "/r2/diagnostics_text", 10
        )

        # Subscribers
        self.serial_rx_sub = self.create_subscription(
            Int16MultiArray,
            "serial_rx_7",
            self.serial_rx_callback,
            10,
            callback_group=callback_group,
        )
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, 10, callback_group=callback_group
        )

        # Startup safety subscribers: monitor serial_tx_ID and serial_rx_ID
        if self.startup_safety_enabled:
            self.refresh_startup_safety_subscriptions()

        # Service servers
        self.run_diagnostics_srv = self.create_service(
            Trigger,
            "/r2/run_diagnostics",
            self.run_diagnostics_callback,
            callback_group=callback_group,
        )
        self.run_hardware_diag_srv = self.create_service(
            Trigger,
            "/r2/run_hardware_diagnostics",
            self.run_hardware_diag_callback,
            callback_group=callback_group,
        )
        self.save_report_srv = self.create_service(
            Trigger,
            "/r2/save_diagnostics_report",
            self.save_report_callback,
            callback_group=callback_group,
        )

        # State variables
        self.last_diagnostics_time = 0.0
        self.diagnostics_interval = 10.0  # Publish diagnostics every 10 seconds
        self.last_report = None

        # Create timer for periodic diagnostics
        self.create_timer(1.0, self.timer_callback, callback_group=callback_group)

        self.get_logger().info("R2 Diagnostics Node initialized")
        self.get_logger().info(
            f"Services available: /r2/run_diagnostics, /r2/run_hardware_diagnostics, /r2/save_diagnostics_report"
        )
        if self.startup_safety_enabled:
            if self.startup_safety_auto_discover:
                self.get_logger().info(
                    f"Startup safety check enabled (window={self.startup_safety_window_sec:.1f}s, device_ids=auto-discover)"
                )
            else:
                ids_text = ", ".join(
                    str(v) for v in sorted(set(self.startup_safety_device_ids))
                )
                self.get_logger().info(
                    f"Startup safety check enabled (window={self.startup_safety_window_sec:.1f}s, device_ids=[{ids_text}])"
                )
            self.get_logger().info(
                f"Startup safety check exit_after_check={self.startup_safety_exit_after_check}"
            )

    def _extract_serial_topic_device_id(self, topic_name: str, prefix: str):
        m = re.fullmatch(rf"/?{prefix}_(\d+)", topic_name)
        if m is None:
            return None
        return int(m.group(1))

    def _create_startup_safety_subscription(self, topic_name: str):
        canonical_topic = topic_name if topic_name.startswith("/") else f"/{topic_name}"
        if canonical_topic in self.startup_safety_subs:
            return

        self.startup_safety_subs[canonical_topic] = self.create_subscription(
            Int16MultiArray,
            canonical_topic,
            lambda msg, topic=canonical_topic: self.serial_startup_safety_callback(
                msg, topic
            ),
            10,
            callback_group=self.callback_group,
        )
        self.startup_safety_monitored_topics.add(canonical_topic)

        tx_id = self._extract_serial_topic_device_id(canonical_topic, "serial_tx")
        rx_id = self._extract_serial_topic_device_id(canonical_topic, "serial_rx")
        if tx_id is not None:
            self.startup_safety_monitored_device_ids.add(tx_id)
        if rx_id is not None:
            self.startup_safety_monitored_device_ids.add(rx_id)

    def refresh_startup_safety_subscriptions(self):
        if not self.startup_safety_enabled or self.startup_safety_completed:
            return

        if self.startup_safety_auto_discover:
            for topic_name, topic_types in self.get_topic_names_and_types():
                if "std_msgs/msg/Int16MultiArray" not in topic_types:
                    continue
                if (
                    self._extract_serial_topic_device_id(topic_name, "serial_tx")
                    is not None
                ):
                    self._create_startup_safety_subscription(topic_name)
                if (
                    self._extract_serial_topic_device_id(topic_name, "serial_rx")
                    is not None
                ):
                    self._create_startup_safety_subscription(topic_name)
            return

        for device_id in sorted(set(self.startup_safety_device_ids)):
            self._create_startup_safety_subscription(f"/serial_tx_{device_id}")
            self._create_startup_safety_subscription(f"/serial_rx_{device_id}")

    def _collect_startup_packet_issues(self, data):
        """Detect potentially dangerous startup packet values."""
        issues = []

        if len(data) == 0:
            return ["empty packet"]

        # At power-on, actuator-related values should stay zero.
        if len(data) > 1 and any(v != 0 for v in data[1:]):
            issues.append("contains non-zero values after startup")

        # Keep range checks to catch malformed values early.
        if data[0] not in (0, 1):
            issues.append(f"index[0]={data[0]} out of range (expected 0 or 1)")

        for i in range(1, min(9, len(data))):
            if abs(data[i]) > 255:
                issues.append(f"MD{i}={data[i]} out of range (-255..255)")

        for i in range(9, min(17, len(data))):
            if data[i] < 0 or data[i] > 270:
                issues.append(f"SERVO{i - 8}={data[i]} out of range (0..270)")

        for i in range(17, min(25, len(data))):
            if data[i] not in (0, 1):
                issues.append(f"TR{i - 16}={data[i]} out of range (0 or 1)")

        return issues

    def serial_startup_safety_callback(self, msg: Int16MultiArray, topic_name: str):
        """Monitor serial packet values right after boot and warn if unsafe."""
        if not self.startup_safety_enabled or self.startup_safety_completed:
            return

        if (
            time.time() - self.startup_safety_start_time
            > self.startup_safety_window_sec
        ):
            return

        self.startup_safety_seen_topics.add(topic_name)
        issues = self._collect_startup_packet_issues(msg.data)
        if not issues:
            return

        if topic_name in self.startup_safety_warned_topics:
            return

        self.startup_safety_warned_topics.add(topic_name)
        self.startup_safety_issue_details[topic_name] = issues

        detail = "; ".join(issues)
        warning = f"[StartupSafety] {topic_name}: {detail}"
        self.get_logger().warn(warning)
        self.diagnostics_text_pub.publish(String(data=warning))

    def serial_rx_callback(self, msg: Int16MultiArray):
        """Handle serial RX data for diagnostics"""
        # Parse serial data and update diagnostics state
        if len(msg.data) >= 24:
            # Expected format: [DEBUG, MD1-8, SERVO1-8, TR1-7]
            # MD motors: indices 1-8
            for i in range(8):
                self.hw_diagnostics.set_motor_state(i + 1, msg.data[1 + i])

            # Servos: indices 9-16
            for i in range(8):
                self.hw_diagnostics.set_servo_state(i + 1, msg.data[9 + i])

            # Solenoids: indices 17-23
            for i in range(7):
                self.hw_diagnostics.set_solenoid_state(i + 1, msg.data[17 + i])

    def joy_callback(self, msg: Joy):
        """Handle joystick input for manual diagnostics trigger"""
        # LB + RB button combination to trigger diagnostics
        if len(msg.buttons) >= 5:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:  # LB and RB
                self.get_logger().info("Diagnostics triggered via joystick")
                self.run_full_diagnostics()

    def timer_callback(self):
        """Periodic timer callback for status publishing"""
        current_time = time.time()

        # Finalize startup safety check once the startup window elapsed.
        if (
            self.startup_safety_enabled
            and not self.startup_safety_completed
            and current_time - self.startup_safety_start_time
            >= self.startup_safety_window_sec
        ):
            self.finalize_startup_safety_check()
        elif self.startup_safety_enabled and not self.startup_safety_completed:
            self.refresh_startup_safety_subscriptions()

        # Publish diagnostics at interval
        if current_time - self.last_diagnostics_time >= self.diagnostics_interval:
            self.publish_diagnostics()
            self.last_diagnostics_time = current_time

    def finalize_startup_safety_check(self):
        """Finalize startup safety verdict after startup window."""
        if self.startup_safety_completed:
            return

        self.startup_safety_completed = True

        expected_topics = set()
        if self.startup_safety_auto_discover:
            for device_id in sorted(self.startup_safety_monitored_device_ids):
                expected_topics.add(f"/serial_tx_{device_id}")
                expected_topics.add(f"/serial_rx_{device_id}")
        else:
            for device_id in sorted(set(self.startup_safety_device_ids)):
                expected_topics.add(f"/serial_tx_{device_id}")
                expected_topics.add(f"/serial_rx_{device_id}")

        missing_topics = sorted(expected_topics - self.startup_safety_seen_topics)
        warned_count = len(self.startup_safety_warned_topics)
        missing_count = len(missing_topics)

        if (
            self.startup_safety_auto_discover
            and not self.startup_safety_monitored_device_ids
        ):
            self.startup_safety_diag_level = DiagnosticStatus.WARN
            self.startup_safety_diag_message = (
                "startup safety check found no serial topics"
            )
        elif self.startup_safety_warned_topics:
            self.startup_safety_diag_level = DiagnosticStatus.WARN
            self.startup_safety_diag_message = (
                f"startup safety warning ({warned_count} topic(s))"
            )
        elif missing_topics:
            self.startup_safety_diag_level = DiagnosticStatus.WARN
            self.startup_safety_diag_message = (
                f"startup safety check incomplete ({missing_count} topic(s) no data)"
            )
        else:
            self.startup_safety_diag_level = DiagnosticStatus.OK
            self.startup_safety_diag_message = (
                "startup safety check passed (all monitored serial topics stayed safe)"
            )

        summary = f"[StartupSafety] {self.startup_safety_diag_message}"
        if self.startup_safety_diag_level == DiagnosticStatus.OK:
            self.get_logger().info(summary)
        else:
            self.get_logger().warn(summary)
        self.diagnostics_text_pub.publish(String(data=summary))
        self._log_startup_safety_report(missing_topics)
        self.publish_diagnostics()

        if self.startup_safety_exit_after_check:
            self._schedule_shutdown()

    def _log_startup_safety_report(self, missing_topics):
        """Print a readable startup safety result to console."""
        monitored_ids = sorted(self.startup_safety_monitored_device_ids)
        self.get_logger().info("[StartupSafety] ===== RESULT =====")

        if self.startup_safety_diag_level == DiagnosticStatus.OK:
            self.get_logger().info("[StartupSafety] STATUS: SAFE")
        else:
            self.get_logger().warn("[StartupSafety] STATUS: WARNING")

        self.get_logger().info(
            f"[StartupSafety] MESSAGE: {self.startup_safety_diag_message}"
        )

        if monitored_ids:
            self.get_logger().info(
                "[StartupSafety] SCANNED_IDS: "
                + ", ".join(str(v) for v in monitored_ids)
            )
            self._log_startup_safety_per_id(monitored_ids)

        if self.startup_safety_issue_details:
            for topic_name in sorted(self.startup_safety_issue_details.keys()):
                detail = "; ".join(self.startup_safety_issue_details[topic_name])
                self.get_logger().warn(f"[StartupSafety] ISSUE {topic_name}: {detail}")

        self.get_logger().info("[StartupSafety] ==================")

    def _topic_state_for_id(self, device_id: int, direction: str):
        topic = f"/serial_{direction}_{device_id}"
        if topic in self.startup_safety_warned_topics:
            return "UNSAFE"
        if topic in self.startup_safety_seen_topics:
            return "SAFE"
        return "NO-DATA"

    def _log_startup_safety_per_id(self, monitored_ids):
        for device_id in monitored_ids:
            tx_state = self._topic_state_for_id(device_id, "tx")
            rx_state = self._topic_state_for_id(device_id, "rx")
            line = f"[StartupSafety] ID {device_id}: tx={tx_state}, rx={rx_state}"
            if "UNSAFE" in (tx_state, rx_state) or "NO-DATA" in (tx_state, rx_state):
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)

    def _schedule_shutdown(self):
        """Shutdown node shortly after printing startup safety result."""
        if self.shutdown_timer is not None:
            return

        self.get_logger().info(
            "[StartupSafety] check completed. shutting down diagnostics node..."
        )
        self.shutdown_timer = self.create_timer(0.2, self._shutdown_once)

    def _shutdown_once(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None
        if rclpy.ok():
            rclpy.shutdown()

    def publish_diagnostics(self):
        """Publish current diagnostics as ROS DiagnosticArray"""
        summary = self.hw_diagnostics.get_summary()

        # Create DiagnosticArray
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Create diagnostic statuses for each result
        for result in summary.results:
            status = DiagnosticStatus()
            status.name = f"r2_{result.component.lower()}_{result.component_id}"

            # Map diagnostic status
            if result.status == HDStatus.PASS:
                status.level = DiagnosticStatus.OK
            elif result.status == HDStatus.WARNING:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR

            status.message = result.message
            status.values.append(KeyValue(key="value", value=str(result.value)))
            diag_array.status.append(status)

        if self.startup_safety_enabled:
            startup_status = DiagnosticStatus()
            startup_status.name = "r2_startup_serial_safety"
            startup_status.level = self.startup_safety_diag_level
            startup_status.message = self.startup_safety_diag_message
            startup_status.values.append(
                KeyValue(
                    key="warned_topics",
                    value=",".join(sorted(self.startup_safety_warned_topics)),
                )
            )
            diag_array.status.append(startup_status)

        # Publish
        self.diagnostic_pub.publish(diag_array)

    def run_full_diagnostics(self):
        """Run full diagnostics"""
        self.get_logger().info("Starting full diagnostics...")

        # Run hardware diagnostics
        summary = self.hw_diagnostics.run_full_diagnostics()
        self.last_report = summary

        # Log results
        self.get_logger().info("\n" + str(summary))

        # Publish as text message
        text_msg = String(data=str(summary))
        self.diagnostics_text_pub.publish(text_msg)

        # Publish as diagnostic array
        self.publish_diagnostics()

    def run_diagnostics_callback(self, request, response):
        """Service callback: Run full diagnostics"""
        try:
            self.run_full_diagnostics()
            response.success = True
            response.message = "Diagnostics completed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Diagnostics failed: {str(e)}"

        return response

    def run_hardware_diag_callback(self, request, response):
        """Service callback: Run hardware diagnostics only"""
        try:
            self.hw_diagnostics.reset()

            # Test all motors
            for i in range(1, 9):
                ok, msg = self.hw_diagnostics.check_motor_response(i)
                status = HDStatus.PASS if ok else HDStatus.FAIL
                self.hw_diagnostics.add_result("Motor", i, status, message=msg)

            # Test all servos
            for i in range(1, 9):
                ok, msg = self.hw_diagnostics.check_servo_response(i)
                status = HDStatus.PASS if ok else HDStatus.FAIL
                self.hw_diagnostics.add_result("Servo", i, status, message=msg)

            # Test all solenoids
            for i in range(1, 8):
                ok, msg = self.hw_diagnostics.check_solenoid_response(i)
                status = HDStatus.PASS if ok else HDStatus.FAIL
                self.hw_diagnostics.add_result("Solenoid", i, status, message=msg)

            summary = self.hw_diagnostics.get_summary()
            self.last_report = summary
            self.get_logger().info("\n" + str(summary))

            response.success = True
            response.message = (
                f"Hardware diagnostics: {summary.passed}/{summary.total_tests} passed"
            )
        except Exception as e:
            response.success = False
            response.message = f"Hardware diagnostics failed: {str(e)}"

        return response

    def save_report_callback(self, request, response):
        """Service callback: Save diagnostics report"""
        try:
            if self.last_report is None:
                response.success = False
                response.message = "No diagnostics report to save"
                return response

            # Create report
            report = create_diagnostics_report(self.last_report)

            # Save to file
            timestamp = int(time.time() * 1000)
            report_dir = Path.home() / ".ros" / "diagnostics_reports"
            report_dir.mkdir(parents=True, exist_ok=True)

            report_file = report_dir / f"diagnostics_report_{timestamp}.json"
            save_diagnostics_report(report, str(report_file))

            response.success = True
            response.message = f"Report saved to {report_file}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save report: {str(e)}"

        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    # Create node
    node = R2DiagnosticsNode()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
