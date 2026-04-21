"""
ROS2 Diagnostics Node for R2 robot
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Int32MultiArray, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Trigger

import time
import json
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

        # Initialize diagnostics
        self.hw_diagnostics = HardwareDiagnostics()
        self.sensor_diagnostics = SensorDiagnostics()

        # Create callback group for threading
        callback_group = ReentrantCallbackGroup()

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

        # Publish diagnostics at interval
        if current_time - self.last_diagnostics_time >= self.diagnostics_interval:
            self.publish_diagnostics()
            self.last_diagnostics_time = current_time

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
            status.values.append(
                DiagnosticStatus.KeyValue(key="value", value=str(result.value))
            )
            diag_array.status.append(status)

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
