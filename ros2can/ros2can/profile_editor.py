"""プロファイル(スロット割り当て)を編集するダイアログ。

CAN_NODE_COUNT / CAN_SLOTS_PER_NODE をファームウェア側で変更した場合や、
スロットのラベル・レンジを実機に合わせて微調整したい場合に使う。
"""

from __future__ import annotations

from typing import List

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit, QSpinBox,
    QPushButton, QTabWidget, QTableWidget, QTableWidgetItem, QComboBox,
    QLabel, QDialogButtonBox, QMessageBox,
)

from .device_profiles import (
    ChannelDef, DeviceProfile, SLOT_COUNT,
    MOTOR, SERVO, DIGITAL_OUT, ENUM_OUT, RAW_OUT,
    COUNTER, DIGITAL_IN, READOUT, ENUM_IN, RAW_IN,
    make_can_host_profile,
)

TX_KINDS = [MOTOR, SERVO, DIGITAL_OUT, ENUM_OUT, RAW_OUT]
RX_KINDS = [COUNTER, DIGITAL_IN, READOUT, ENUM_IN, RAW_IN]

COLUMNS = ["index", "label", "kind", "group", "min", "max", "unit", "scale", "note"]


class _SlotTable(QTableWidget):
    def __init__(self, kinds: List[str], parent=None):
        super().__init__(SLOT_COUNT, len(COLUMNS), parent)
        self.kinds = kinds
        self.setHorizontalHeaderLabels(COLUMNS)
        self.verticalHeader().setVisible(False)
        for i in range(SLOT_COUNT):
            idx_item = QTableWidgetItem(str(i))
            idx_item.setFlags(idx_item.flags() & ~Qt.ItemIsEditable)
            self.setItem(i, 0, idx_item)
            self.setItem(i, 1, QTableWidgetItem(f"slot{i}"))

            combo = QComboBox()
            combo.addItems(kinds)
            self.setCellWidget(i, 2, combo)

            self.setItem(i, 3, QTableWidgetItem(""))
            self.setItem(i, 4, QTableWidgetItem("-32768"))
            self.setItem(i, 5, QTableWidgetItem("32767"))
            self.setItem(i, 6, QTableWidgetItem(""))
            self.setItem(i, 7, QTableWidgetItem("1.0"))
            self.setItem(i, 8, QTableWidgetItem(""))

        self.resizeColumnsToContents()

    def load(self, defs: List[ChannelDef]) -> None:
        by_index = {c.index: c for c in defs}
        for i in range(SLOT_COUNT):
            c = by_index.get(i)
            if c is None:
                continue
            self.item(i, 1).setText(c.label)
            combo: QComboBox = self.cellWidget(i, 2)
            idx = combo.findText(c.kind)
            if idx >= 0:
                combo.setCurrentIndex(idx)
            self.item(i, 3).setText(c.group)
            self.item(i, 4).setText(str(c.min))
            self.item(i, 5).setText(str(c.max))
            self.item(i, 6).setText(c.unit)
            self.item(i, 7).setText(str(c.scale))
            self.item(i, 8).setText(c.note)

    def collect(self) -> List[ChannelDef]:
        result = []
        for i in range(SLOT_COUNT):
            label = self.item(i, 1).text().strip() or f"slot{i}"
            kind = self.cellWidget(i, 2).currentText()
            group = self.item(i, 3).text().strip()
            try:
                min_v = int(float(self.item(i, 4).text()))
            except ValueError:
                min_v = -32768
            try:
                max_v = int(float(self.item(i, 5).text()))
            except ValueError:
                max_v = 32767
            unit = self.item(i, 6).text().strip()
            try:
                scale = float(self.item(i, 7).text())
            except ValueError:
                scale = 1.0
            note = self.item(i, 8).text().strip()
            result.append(ChannelDef(
                index=i, label=label, kind=kind, group=group,
                min=min_v, max=max_v, unit=unit, scale=scale, note=note,
            ))
        return result


class ProfileEditorDialog(QDialog):
    def __init__(self, base_profile: DeviceProfile, parent=None):
        super().__init__(parent)
        self.setWindowTitle("プロファイル編集")
        self.resize(900, 560)
        self._profile = base_profile

        layout = QVBoxLayout(self)

        form = QFormLayout()
        self.name_edit = QLineEdit(base_profile.name)
        form.addRow("プロファイル名:", self.name_edit)

        gen_row = QHBoxLayout()
        self.node_count_spin = QSpinBox()
        self.node_count_spin.setRange(0, 24)
        self.node_count_spin.setValue(base_profile.node_count)
        self.slots_per_node_spin = QSpinBox()
        self.slots_per_node_spin.setRange(1, 24)
        self.slots_per_node_spin.setValue(base_profile.slots_per_node or 5)
        gen_btn = QPushButton("この構成でノード用スロットを自動生成")
        gen_btn.clicked.connect(self._on_generate)
        gen_row.addWidget(QLabel("CANノード数:"))
        gen_row.addWidget(self.node_count_spin)
        gen_row.addWidget(QLabel("ノードあたりスロット数:"))
        gen_row.addWidget(self.slots_per_node_spin)
        gen_row.addWidget(gen_btn)
        gen_row.addStretch(1)

        layout.addLayout(form)
        layout.addLayout(gen_row)
        layout.addWidget(QLabel(
            "自動生成: 各ノードの先頭4スロットをMD1-4(指令)/SW1-3+ENC1(帰還)、"
            "5番目のスロットをSERVO1(指令)/ENC2(帰還)として割り当てます。"
            "個別のラベルやレンジは下の表で自由に編集できます。"))

        self.tabs = QTabWidget()
        self.tx_table = _SlotTable(TX_KINDS)
        self.rx_table = _SlotTable(RX_KINDS)
        self.tabs.addTab(self.tx_table, "TX (ROS -> マイコン)")
        self.tabs.addTab(self.rx_table, "RX (マイコン -> ROS)")
        layout.addWidget(self.tabs)

        self.tx_table.load(base_profile.tx)
        self.rx_table.load(base_profile.rx)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._on_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _on_generate(self) -> None:
        node_count = self.node_count_spin.value()
        slots_per_node = self.slots_per_node_spin.value()
        if node_count * slots_per_node > SLOT_COUNT:
            QMessageBox.warning(
                self, "スロット数超過",
                f"ノード数 x スロット数 ({node_count * slots_per_node}) が"
                f"24スロットを超えています。")
            return
        generated = make_can_host_profile(
            key=self._profile.key, name=self.name_edit.text(),
            node_count=node_count, slots_per_node=slots_per_node)
        self.tx_table.load(generated.tx)
        self.rx_table.load(generated.rx)

    def _on_accept(self) -> None:
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "入力エラー", "プロファイル名を入力してください。")
            return
        self._profile.name = name
        self._profile.node_count = self.node_count_spin.value()
        self._profile.slots_per_node = self.slots_per_node_spin.value()
        self._profile.tx = self.tx_table.collect()
        self._profile.rx = self.rx_table.collect()
        self._profile.editable = True
        self.accept()

    def result_profile(self) -> DeviceProfile:
        return self._profile
