"""再利用可能な小さな Qt ウィジェット群。"""

from __future__ import annotations

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QLabel, QSlider, QDoubleSpinBox, QHBoxLayout,
    QPushButton, QComboBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QSpinBox,
)

from .device_profiles import (
    ChannelDef, DIGITAL_OUT, ENUM_OUT,
    DIGITAL_IN, ENUM_IN, SLOT_COUNT,
)


class LedIndicator(QLabel):
    """ON/OFF を色付きの丸で表示する軽量インジケータ。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(18, 18)
        self.set_state(False)

    def set_state(self, on: bool) -> None:
        color = "#2ecc71" if on else "#555555"
        self.setStyleSheet(
            f"background-color:{color}; border-radius:9px; border:1px solid #222;"
        )


class ChannelControlRow(QWidget):
    """TX (ROS -> マイコン) の 1 チャンネル分の指令値入力行。"""

    valueChanged = pyqtSignal(int, int)  # (slot_index, raw_value)

    def __init__(self, chdef: ChannelDef, parent=None):
        super().__init__(parent)
        self.chdef = chdef
        self._raw_value = 0
        self._updating = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)

        label_text = f"[{chdef.index}] {chdef.label}"
        name_label = QLabel(label_text)
        name_label.setMinimumWidth(160)
        if chdef.note:
            name_label.setToolTip(chdef.note)
        layout.addWidget(name_label)

        if chdef.kind == DIGITAL_OUT:
            self.toggle_btn = QPushButton("OFF")
            self.toggle_btn.setCheckable(True)
            self.toggle_btn.setMinimumWidth(80)
            self.toggle_btn.toggled.connect(self._on_toggle)
            layout.addWidget(self.toggle_btn)
            layout.addStretch(1)

        elif chdef.kind == ENUM_OUT:
            self.combo = QComboBox()
            for raw, text in (chdef.options or []):
                self.combo.addItem(text, raw)
            self.combo.currentIndexChanged.connect(self._on_combo)
            layout.addWidget(self.combo)
            layout.addStretch(1)

        else:
            # motor / servo / raw_out (数値スライダー+スピンボックス)
            self.slider = QSlider(Qt.Horizontal)
            self.slider.setMinimum(chdef.min)
            self.slider.setMaximum(chdef.max)
            self.slider.setValue(0 if chdef.min <= 0 <= chdef.max else chdef.min)
            self.slider.valueChanged.connect(self._on_slider)
            layout.addWidget(self.slider, 3)

            self.spin = QDoubleSpinBox()
            self.spin.setDecimals(chdef.decimals if chdef.scale != 1.0 else 0)
            self.spin.setMinimum(chdef.min * chdef.scale)
            self.spin.setMaximum(chdef.max * chdef.scale)
            self.spin.setSingleStep(chdef.step * chdef.scale if chdef.step else 1)
            if chdef.unit:
                self.spin.setSuffix(f" {chdef.unit}")
            self.spin.setValue(self.slider.value() * chdef.scale)
            self.spin.valueChanged.connect(self._on_spin)
            self.spin.setMinimumWidth(110)
            layout.addWidget(self.spin)

        self.setLayout(layout)

    def _emit(self, raw: int) -> None:
        raw = int(max(self.chdef.min, min(self.chdef.max, raw)))
        self._raw_value = raw
        self.valueChanged.emit(self.chdef.index, raw)

    def _on_toggle(self, checked: bool) -> None:
        self.toggle_btn.setText("ON" if checked else "OFF")
        self.toggle_btn.setStyleSheet(
            "background-color:#2ecc71;" if checked else ""
        )
        if not self._updating:
            self._emit(1 if checked else 0)

    def _on_combo(self, _idx: int) -> None:
        if self._updating:
            return
        raw = self.combo.currentData()
        self._emit(int(raw) if raw is not None else 0)

    def _on_slider(self, value: int) -> None:
        if self._updating:
            return
        self._updating = True
        self.spin.setValue(value * self.chdef.scale)
        self._updating = False
        self._emit(value)

    def _on_spin(self, value: float) -> None:
        if self._updating:
            return
        raw = self.chdef.raw_from_display(value)
        self._updating = True
        self.slider.setValue(int(max(self.chdef.min, min(self.chdef.max, raw))))
        self._updating = False
        self._emit(raw)

    def set_raw_value(self, raw: int) -> None:
        """外部(Rawタブなど)からの反映。シグナルは発行しない。"""
        self._updating = True
        try:
            if self.chdef.kind == DIGITAL_OUT:
                self.toggle_btn.setChecked(bool(raw))
            elif self.chdef.kind == ENUM_OUT:
                idx = self.combo.findData(raw)
                if idx >= 0:
                    self.combo.setCurrentIndex(idx)
            else:
                self.slider.setValue(int(max(self.chdef.min, min(self.chdef.max, raw))))
                self.spin.setValue(raw * self.chdef.scale)
        finally:
            self._updating = False
        self._raw_value = raw


class ChannelMonitorRow(QWidget):
    """RX (マイコン -> ROS) の 1 チャンネル分のセンサ値表示行。"""

    def __init__(self, chdef: ChannelDef, parent=None):
        super().__init__(parent)
        self.chdef = chdef

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)

        label_text = f"[{chdef.index}] {chdef.label}"
        name_label = QLabel(label_text)
        name_label.setMinimumWidth(160)
        if chdef.note:
            name_label.setToolTip(chdef.note)
        layout.addWidget(name_label)

        if chdef.kind == DIGITAL_IN:
            self.led = LedIndicator()
            layout.addWidget(self.led)
            layout.addStretch(1)
            self.value_label = None
        else:
            self.value_label = QLabel("0")
            self.value_label.setStyleSheet("font-weight:bold; font-size:13pt;")
            self.value_label.setMinimumWidth(100)
            layout.addWidget(self.value_label)
            if chdef.unit:
                layout.addWidget(QLabel(chdef.unit))
            layout.addStretch(1)
            self.led = None

        self.setLayout(layout)

    def set_raw_value(self, raw: int) -> None:
        if self.chdef.kind == DIGITAL_IN:
            self.led.set_state(bool(raw))
        elif self.chdef.kind == ENUM_IN:
            text = None
            for opt_raw, opt_text in (self.chdef.options or []):
                if opt_raw == raw:
                    text = opt_text
                    break
            self.value_label.setText(text if text is not None else str(raw))
        else:
            display = self.chdef.display_value(raw)
            if self.chdef.scale != 1.0:
                self.value_label.setText(f"{display:.{max(1, self.chdef.decimals)}f}")
            else:
                self.value_label.setText(str(raw))


class RawSlotTable(QTableWidget):
    """全24スロットを生の int16 として一覧表示/編集するテーブル。"""

    valueChanged = pyqtSignal(int, int)  # (slot_index, raw_value)

    def __init__(self, editable: bool, parent=None):
        super().__init__(SLOT_COUNT, 3, parent)
        self.editable = editable
        self.setHorizontalHeaderLabels(["slot", "label", "value"])
        self.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self._spins = {}

        for i in range(SLOT_COUNT):
            idx_item = QTableWidgetItem(str(i))
            idx_item.setFlags(idx_item.flags() & ~Qt.ItemIsEditable)
            self.setItem(i, 0, idx_item)

            label_item = QTableWidgetItem("")
            label_item.setFlags(label_item.flags() & ~Qt.ItemIsEditable)
            self.setItem(i, 1, label_item)

            if editable:
                spin = QSpinBox()
                spin.setRange(-32768, 32767)
                spin.valueChanged.connect(
                    lambda v, row=i: self.valueChanged.emit(row, v))
                self.setCellWidget(i, 2, spin)
                self._spins[i] = spin
            else:
                value_item = QTableWidgetItem("0")
                value_item.setFlags(value_item.flags() & ~Qt.ItemIsEditable)
                self.setItem(i, 2, value_item)

        self.setColumnWidth(0, 40)
        self.setColumnWidth(2, 120)

    def set_labels(self, labels: dict) -> None:
        for i in range(SLOT_COUNT):
            self.item(i, 1).setText(labels.get(i, ""))

    def set_values(self, values) -> None:
        for i, v in enumerate(values[:SLOT_COUNT]):
            if self.editable:
                spin = self._spins[i]
                if spin.value() != v:
                    spin.blockSignals(True)
                    spin.setValue(v)
                    spin.blockSignals(False)
            else:
                self.item(i, 2).setText(str(v))

    def set_value_silent(self, index: int, value: int) -> None:
        if not self.editable:
            return
        spin = self._spins[index]
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)
