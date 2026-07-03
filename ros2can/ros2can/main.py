"""ros2can エントリポイント。

Qt のイベントループをメインループとし、QTimer から高頻度に
`rclpy.spin_once(timeout_sec=0)` を呼び出すことで ROS のコールバックを
GUI スレッド上でシリアルに処理する。これによりスレッド間排他を持ち込まずに
Qt シグナルを安全に発行できる。
"""

from __future__ import annotations

import sys

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .ros_backend import RosBackend
from .main_window import MainWindow

SPIN_INTERVAL_MS = 10
PUBLISH_INTERVAL_MS = 50  # armed デバイスへの周期送信 (20Hz)


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)

    app = QApplication(sys.argv)
    app.setApplicationName("ros2can")

    backend = RosBackend()
    window = MainWindow(backend)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(backend.spin_once)
    spin_timer.start(SPIN_INTERVAL_MS)

    publish_timer = QTimer()
    publish_timer.timeout.connect(backend.publish_all_armed)
    publish_timer.start(PUBLISH_INTERVAL_MS)

    exit_code = app.exec_()

    backend.emergency_stop_all()
    backend.shutdown()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
