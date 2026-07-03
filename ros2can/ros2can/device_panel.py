"""CANホスト1台分のデバッグパネル (Control / Monitor / Raw / Info タブ)。

Control / Monitor タブは、プロファイルが CAN ノード構成 (node_count > 0) を
持つ場合、ノードごとのサブタブとして表示する。これにより「どのマイコン
(CANノード)を操作するか」をタブ切り替えで直感的に選択できる。
"""

from __future__ import annotations

import copy
from typing import Dict

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QTabWidget,
    QScrollArea, QComboBox, QPushButton, QCheckBox, QPlainTextEdit,
)

from .device_profiles import (
    all_profiles, DeviceProfile, RAW_OUT, RAW_IN,
    save_custom_profile, unique_custom_key,
)
from .ros_backend import RosBackend, DeviceChannel
from .widgets import ChannelControlRow, ChannelMonitorRow, RawSlotTable, LedIndicator
from .profile_editor import ProfileEditorDialog


def _clear_layout(layout) -> None:
    while layout.count():
        item = layout.takeAt(0)
        w = item.widget()
        if w is not None:
            w.setParent(None)
            w.deleteLater()


class DevicePanel(QWidget):
    def __init__(self, backend: RosBackend, device_id: int, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.device_id = device_id
        self.channel: DeviceChannel = backend.devices[device_id]
        self.control_rows: Dict[int, ChannelControlRow] = {}
        self.monitor_rows: Dict[int, ChannelMonitorRow] = {}

        self._build_header()

        self.tabs = QTabWidget()

        self.control_host = QWidget()
        self.control_host_layout = QVBoxLayout(self.control_host)
        self.control_host_layout.setContentsMargins(0, 0, 0, 0)

        self.monitor_host = QWidget()
        self.monitor_host_layout = QVBoxLayout(self.monitor_host)
        self.monitor_host_layout.setContentsMargins(0, 0, 0, 0)

        self.tabs.addTab(self.control_host, "Control (指令送信)")
        self.tabs.addTab(self.monitor_host, "Monitor (センサ受信)")

        self.raw_tx_table = RawSlotTable(editable=True)
        self.raw_rx_table = RawSlotTable(editable=False)
        raw_widget = QWidget()
        raw_layout = QHBoxLayout(raw_widget)
        tx_box = QVBoxLayout()
        tx_box.addWidget(QLabel("TX (ROS -> ホスト -> CAN, 編集可能)"))
        tx_box.addWidget(self.raw_tx_table)
        rx_box = QVBoxLayout()
        rx_box.addWidget(QLabel("RX (CAN -> ホスト -> ROS, 読み取り専用)"))
        rx_box.addWidget(self.raw_rx_table)
        raw_layout.addLayout(tx_box)
        raw_layout.addLayout(rx_box)
        self.tabs.addTab(raw_widget, "Raw (全24スロット)")

        self.info_text = QPlainTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setStyleSheet("font-family: monospace;")
        self.tabs.addTab(self.info_text, "Info")

        self.raw_tx_table.valueChanged.connect(self._on_raw_tx_changed)

        outer = QVBoxLayout(self)
        outer.addWidget(self.header)
        outer.addWidget(self.tabs)

        self._rebuild_for_profile()
        self.refresh_from_rx()

    # ---------------- header ----------------

    def _build_header(self) -> None:
        self.header = QGroupBox()
        layout = QHBoxLayout(self.header)

        title = QLabel(f"<b>Device ID {self.device_id}</b> (CANホスト)")
        title.setStyleSheet("font-size:13pt;")
        layout.addWidget(title)

        self.led = LedIndicator()
        layout.addWidget(self.led)
        self.status_label = QLabel("未接続")
        layout.addWidget(self.status_label)

        layout.addSpacing(20)
        layout.addWidget(QLabel("プロファイル:"))
        self.profile_combo = QComboBox()
        self.profile_combo.setMinimumWidth(300)
        self._reload_profile_list()
        self.profile_combo.currentIndexChanged.connect(self._on_profile_selected)
        layout.addWidget(self.profile_combo)

        self.edit_profile_btn = QPushButton("プロファイル編集")
        self.edit_profile_btn.clicked.connect(self._on_edit_profile)
        layout.addWidget(self.edit_profile_btn)

        layout.addStretch(1)

        self.armed_check = QCheckBox("TX有効化 (実際にマイコンへ送信)")
        self.armed_check.setStyleSheet("QCheckBox { font-weight: bold; }")
        self.armed_check.toggled.connect(self._on_armed_toggled)
        layout.addWidget(self.armed_check)

        self.zero_btn = QPushButton("全スロットを0にして送信")
        self.zero_btn.setStyleSheet("background-color:#c0392b; color:white; font-weight:bold;")
        self.zero_btn.clicked.connect(self._on_zero_clicked)
        layout.addWidget(self.zero_btn)

    def _reload_profile_list(self) -> None:
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        profiles = all_profiles()
        keys = sorted(profiles.keys(), key=lambda k: profiles[k].name)
        current_index = 0
        for i, key in enumerate(keys):
            self.profile_combo.addItem(profiles[key].name, key)
            self.profile_combo.setItemData(i, profiles[key].description, 3)  # Qt.ToolTipRole
            if key == self.channel.profile_key:
                current_index = i
        self.profile_combo.setCurrentIndex(current_index)
        self.profile_combo.blockSignals(False)

    def current_profile(self) -> DeviceProfile:
        profiles = all_profiles()
        return profiles.get(self.channel.profile_key, profiles["generic_raw"])

    # ---------------- profile change / rebuild ----------------

    def _on_profile_selected(self, _index: int) -> None:
        key = self.profile_combo.currentData()
        if key:
            self.channel.profile_key = key
            self._rebuild_for_profile()
            self.refresh_from_rx()

    def _on_edit_profile(self) -> None:
        base = self.current_profile()
        new_profile = copy.deepcopy(base)
        existing = all_profiles()
        if not base.editable:
            new_profile.key = unique_custom_key(base.key, existing)
            new_profile.name = f"{base.name} (カスタム)"
        dlg = ProfileEditorDialog(new_profile, self)
        if dlg.exec_():
            result = dlg.result_profile()
            save_custom_profile(result)
            self.channel.profile_key = result.key
            self._reload_profile_list()
            self._rebuild_for_profile()
            self.refresh_from_rx()

    def _build_channel_rows(self, defs, row_cls, on_change=None):
        """group名ごとにまとめた縦並びウィジェットを1つ作る。"""
        widget = QWidget()
        v = QVBoxLayout(widget)
        v.setContentsMargins(4, 4, 4, 4)
        rows = {}
        for chdef in sorted(defs, key=lambda c: c.index):
            row = row_cls(chdef)
            if on_change is not None:
                row.valueChanged.connect(on_change)
            v.addWidget(row)
            rows[chdef.index] = row
        v.addStretch(1)
        return widget, rows

    def _rebuild_for_profile(self) -> None:
        profile = self.current_profile()
        self.control_rows.clear()
        self.monitor_rows.clear()
        _clear_layout(self.control_host_layout)
        _clear_layout(self.monitor_host_layout)

        defined_tx = [c for c in profile.tx if c.kind != RAW_OUT]
        defined_rx = [c for c in profile.rx if c.kind != RAW_IN]

        if profile.node_count > 0:
            # ノードごとのサブタブとして「どのマイコンを操作するか」を選択できるようにする
            control_tabs = QTabWidget()
            monitor_tabs = QTabWidget()
            for node in range(profile.node_count):
                base = node * profile.slots_per_node
                node_no = node + 1
                node_tx = [c for c in defined_tx if base <= c.index < base + profile.slots_per_node]
                node_rx = [c for c in defined_rx if base <= c.index < base + profile.slots_per_node]

                cw, crows = self._build_channel_rows(
                    node_tx, ChannelControlRow, self._on_control_value_changed)
                self.control_rows.update(crows)
                scroll_c = QScrollArea()
                scroll_c.setWidgetResizable(True)
                scroll_c.setWidget(cw)
                control_tabs.addTab(scroll_c, f"ノード{node_no}")

                mw, mrows = self._build_channel_rows(node_rx, ChannelMonitorRow)
                self.monitor_rows.update(mrows)
                scroll_m = QScrollArea()
                scroll_m.setWidgetResizable(True)
                scroll_m.setWidget(mw)
                monitor_tabs.addTab(scroll_m, f"ノード{node_no}")

            self.control_host_layout.addWidget(control_tabs)
            self.monitor_host_layout.addWidget(monitor_tabs)
        else:
            # ノード構成を持たないプロファイル (汎用Raw等): groupごとに縦積み表示
            cw, crows = self._grouped_widget(defined_tx, ChannelControlRow,
                                              self._on_control_value_changed,
                                              "このプロファイルに個別定義された送信スロットはありません。"
                                              "Raw タブで全24スロットを直接編集してください。")
            self.control_rows.update(crows)
            scroll_c = QScrollArea()
            scroll_c.setWidgetResizable(True)
            scroll_c.setWidget(cw)
            self.control_host_layout.addWidget(scroll_c)

            mw, mrows = self._grouped_widget(defined_rx, ChannelMonitorRow, None,
                                              "このプロファイルに個別定義された受信スロットはありません。"
                                              "Raw タブで全24スロットを直接確認してください。")
            self.monitor_rows.update(mrows)
            scroll_m = QScrollArea()
            scroll_m.setWidgetResizable(True)
            scroll_m.setWidget(mw)
            self.monitor_host_layout.addWidget(scroll_m)

        # --- Raw tab labels ---
        tx_labels = {c.index: c.label for c in profile.tx}
        rx_labels = {c.index: c.label for c in profile.rx}
        self.raw_tx_table.set_labels(tx_labels)
        self.raw_rx_table.set_labels(rx_labels)
        self.raw_tx_table.set_values(self.channel.tx_data)
        self.raw_rx_table.set_values(self.channel.rx_data)

        # 現在の tx_data を新しい Control 行にも反映
        for index, row in self.control_rows.items():
            row.set_raw_value(self.channel.tx_data[index])

    def _grouped_widget(self, defs, row_cls, on_change, empty_message: str):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        groups: Dict[str, QVBoxLayout] = {}
        rows = {}
        for chdef in sorted(defs, key=lambda c: c.index):
            group_name = chdef.group or "その他"
            if group_name not in groups:
                box = QGroupBox(group_name)
                v = QVBoxLayout(box)
                groups[group_name] = v
                layout.addWidget(box)
            row = row_cls(chdef)
            if on_change is not None:
                row.valueChanged.connect(on_change)
            groups[group_name].addWidget(row)
            rows[chdef.index] = row
        if not defs:
            layout.addWidget(QLabel(empty_message))
        layout.addStretch(1)
        return widget, rows

    # ---------------- value flow ----------------

    def _on_control_value_changed(self, index: int, raw: int) -> None:
        self.channel.tx_data[index] = raw
        self.raw_tx_table.set_value_silent(index, raw)

    def _on_raw_tx_changed(self, index: int, raw: int) -> None:
        self.channel.tx_data[index] = raw
        row = self.control_rows.get(index)
        if row is not None:
            row.set_raw_value(raw)

    def _on_armed_toggled(self, checked: bool) -> None:
        self.channel.armed = checked
        self.armed_check.setStyleSheet(
            "QCheckBox { font-weight: bold; color: #c0392b; }" if checked
            else "QCheckBox { font-weight: bold; }")

    def _on_zero_clicked(self) -> None:
        self.backend.zero_and_send(self.device_id)
        for row in self.control_rows.values():
            row.set_raw_value(0)
        self.raw_tx_table.set_values(self.channel.tx_data)

    def set_armed_external(self, armed: bool) -> None:
        """E-STOP などパネル外からの状態反映。"""
        self.armed_check.blockSignals(True)
        self.armed_check.setChecked(armed)
        self.armed_check.blockSignals(False)
        self._on_armed_toggled(armed)
        for row in self.control_rows.values():
            row.set_raw_value(0)
        self.raw_tx_table.set_values(self.channel.tx_data)

    # ---------------- rx refresh ----------------

    def refresh_from_rx(self) -> None:
        ch = self.channel
        self.led.set_state(ch.connected)
        self.status_label.setText(
            f"{'接続中' if ch.connected else '未接続'}  RX {ch.rx_hz:.1f}Hz")

        for index, row in self.monitor_rows.items():
            row.set_raw_value(ch.rx_data[index])
        self.raw_rx_table.set_values(ch.rx_data)

        info_lines = [
            f"device_id       : {self.device_id}",
            f"profile         : {self.channel.profile_key}",
            f"manual_add      : {ch.manual}",
            f"connected       : {ch.connected}",
            f"rx_hz           : {ch.rx_hz:.2f}",
            f"rx_frame_count  : {ch.rx_frame_count}",
            f"tx_frame_count  : {ch.tx_frame_count}",
            f"armed (tx send) : {ch.armed}",
            "",
            f"tx_data (24) = {ch.tx_data}",
            f"rx_data (24) = {ch.rx_data}",
        ]
        self.info_text.setPlainText("\n".join(info_lines))
