"""rclpy と PyQt5 をつなぐバックエンド。

GUI は単一スレッドで動作させる。QTimer から `rclpy.spin_once(timeout_sec=0)` を
高頻度に呼び出すことで ROS のコールバック(サブスクライバ受信)を GUI スレッド上で
直接処理し、スレッド間排他を持ち込まずに Qt シグナルを発行できるようにしている。
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from PyQt5.QtCore import QObject, pyqtSignal

from .device_profiles import DEFAULT_PROFILE_KEY, SLOT_COUNT

TOPIC_RE = re.compile(r"^/?(serial_tx|serial_rx)_(\d+)$")

# この時間 [s] 以上 RX が無ければ「オフライン」とみなす (表示用のヒューリスティック)
STALE_TIMEOUT_SEC = 1.5


@dataclass
class DeviceChannel:
    device_id: int
    profile_key: str = DEFAULT_PROFILE_KEY
    tx_data: List[int] = field(default_factory=lambda: [0] * SLOT_COUNT)
    rx_data: List[int] = field(default_factory=lambda: [0] * SLOT_COUNT)
    armed: bool = False
    manual: bool = False
    last_rx_time: Optional[float] = None
    rx_frame_count: int = 0
    tx_frame_count: int = 0
    _rx_times: List[float] = field(default_factory=list)
    publisher = None
    subscription = None

    @property
    def connected(self) -> bool:
        if self.last_rx_time is None:
            return False
        return (time.monotonic() - self.last_rx_time) < STALE_TIMEOUT_SEC

    @property
    def rx_hz(self) -> float:
        now = time.monotonic()
        self._rx_times = [t for t in self._rx_times if now - t < 2.0]
        if len(self._rx_times) < 2:
            return 0.0
        span = self._rx_times[-1] - self._rx_times[0]
        if span <= 0:
            return 0.0
        return (len(self._rx_times) - 1) / span

    def note_rx(self) -> None:
        now = time.monotonic()
        self.last_rx_time = now
        self.rx_frame_count += 1
        self._rx_times.append(now)


class RosBackend(QObject):
    deviceListChanged = pyqtSignal()
    rxUpdated = pyqtSignal(int)

    def __init__(self, node_name: str = "ros2can_gui"):
        super().__init__()
        self.node: Node = rclpy.create_node(node_name)
        self.devices: Dict[int, DeviceChannel] = {}

    # ---------------- topic discovery ----------------

    def rescan_topics(self) -> None:
        found_ids = set()
        try:
            topics = self.node.get_topic_names_and_types()
        except Exception:
            return

        for name, _types in topics:
            m = TOPIC_RE.match(name)
            if m:
                found_ids.add(int(m.group(2)))

        changed = False
        for device_id in sorted(found_ids):
            if device_id not in self.devices:
                self.add_device(device_id, manual=False)
                changed = True

        if changed:
            self.deviceListChanged.emit()

    # ---------------- device management ----------------

    def add_device(self, device_id: int, manual: bool = True) -> DeviceChannel:
        if device_id in self.devices:
            return self.devices[device_id]

        ch = DeviceChannel(device_id=device_id, manual=manual)
        ch.publisher = self.node.create_publisher(
            Int16MultiArray, f"serial_tx_{device_id}", 10)
        ch.subscription = self.node.create_subscription(
            Int16MultiArray, f"serial_rx_{device_id}",
            lambda msg, did=device_id: self._on_rx(did, msg), 10)
        self.devices[device_id] = ch
        self.deviceListChanged.emit()
        return ch

    def remove_device(self, device_id: int) -> None:
        ch = self.devices.pop(device_id, None)
        if ch is None:
            return
        if ch.publisher is not None:
            self.node.destroy_publisher(ch.publisher)
        if ch.subscription is not None:
            self.node.destroy_subscription(ch.subscription)
        self.deviceListChanged.emit()

    def _on_rx(self, device_id: int, msg: Int16MultiArray) -> None:
        ch = self.devices.get(device_id)
        if ch is None:
            return
        data = list(msg.data[:SLOT_COUNT])
        if len(data) < SLOT_COUNT:
            data += [0] * (SLOT_COUNT - len(data))
        ch.rx_data = data
        ch.note_rx()
        self.rxUpdated.emit(device_id)

    # ---------------- tx ----------------

    def publish_tx(self, device_id: int) -> None:
        ch = self.devices.get(device_id)
        if ch is None or ch.publisher is None:
            return
        msg = Int16MultiArray()
        msg.data = [int(v) for v in ch.tx_data]
        ch.publisher.publish(msg)
        ch.tx_frame_count += 1

    def publish_all_armed(self) -> None:
        for device_id, ch in self.devices.items():
            if ch.armed:
                self.publish_tx(device_id)

    def zero_and_send(self, device_id: int) -> None:
        ch = self.devices.get(device_id)
        if ch is None:
            return
        ch.tx_data = [0] * SLOT_COUNT
        self.publish_tx(device_id)

    def emergency_stop_all(self) -> None:
        for device_id, ch in self.devices.items():
            ch.armed = False
            ch.tx_data = [0] * SLOT_COUNT
            self.publish_tx(device_id)

    # ---------------- lifecycle ----------------

    def spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0)

    def shutdown(self) -> None:
        for device_id in list(self.devices.keys()):
            self.remove_device(device_id)
        self.node.destroy_node()
