"""xiao_esp32_s3_smd_serial_bridge (MODE_CAN_HOST) のスロット割り当て定義。

この基板は USB-シリアルで ROS 2 (serial_bridge) とつながる「CANホスト」であり、
自身の配下に CAN バス経由で最大 CAN_NODE_COUNT 台の子マイコン(ノード)をぶら下げ、
1ノードあたり CAN_SLOTS_PER_NODE 個のスロットを担当させる。
ホストは USB 側では通常の serial_bridge フレーム (24 x int16, TX/RX) を使い、
その 24 スロットを CAN バス上の各ノードへ分配/集約する。

[xiao_esp32_s3_smd_serial_bridge/src/frame_data.hpp, can_task.cpp, config.hpp より]

  ノードごとの担当スロット (CAN_SLOTS_PER_NODE = 5):
    指令 (ROS -> ホスト -> CAN -> ノード):
      0: MD1, 1: MD2, 2: MD3, 3: MD4, 4: SERVO1
    帰還 (ノード -> CAN -> ホスト -> ROS):
      0: SW1, 1: SW2, 2: SW3, 3: ENC1, 4: ENC2

  グローバルスロット index = node_index * CAN_SLOTS_PER_NODE + local_index
  (node_index は 0-origin。ノードの CAN_ID は 101,102,103,104 のように
   下2桁が (node_index+1) になるよう設定する: CAN_NODE_INDEX = (CAN_ID % 100) - 1)

config.hpp で CAN_NODE_COUNT / CAN_SLOTS_PER_NODE を変更した場合は、
GUI の「プロファイル編集」でノード数/スロット数を調整したカスタムプロファイルを
作成すること。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

SLOT_COUNT = 24

# ファームウェア既定値 (config.hpp)
DEFAULT_CAN_NODE_COUNT = 4
DEFAULT_CAN_SLOTS_PER_NODE = 5

# --- 送信 (ROS -> ホスト -> CAN -> ノード, actuator command) ---
MOTOR = "motor"          # 双極 PWM 指令 (DCモータ)
SERVO = "servo"          # サーボ角度指令 [deg]
DIGITAL_OUT = "digital_out"  # ON/OFF 出力
ENUM_OUT = "enum_out"     # 選択式の指令値
RAW_OUT = "raw_out"       # 生の int16 指令値

# --- 受信 (ノード -> CAN -> ホスト -> ROS, sensor feedback) ---
COUNTER = "counter"       # 符号付きカウンタ (エンコーダ等)
DIGITAL_IN = "digital_in"  # ON/OFF 入力 (スイッチ等)
READOUT = "readout"       # スケール付き数値表示
ENUM_IN = "enum_in"       # 選択肢付き数値表示
RAW_IN = "raw_in"         # 生の int16 値表示


@dataclass
class ChannelDef:
    index: int
    label: str
    kind: str
    group: str = ""
    min: int = -32768
    max: int = 32767
    step: int = 1
    unit: str = ""
    scale: float = 1.0
    decimals: int = 0
    options: Optional[List[Tuple[int, str]]] = None
    note: str = ""

    def display_value(self, raw: int) -> float:
        return raw * self.scale

    def raw_from_display(self, value: float) -> int:
        if self.scale == 0:
            return int(value)
        return int(round(value / self.scale))


@dataclass
class DeviceProfile:
    key: str
    name: str
    description: str = ""
    tx: List[ChannelDef] = field(default_factory=list)
    rx: List[ChannelDef] = field(default_factory=list)
    node_count: int = 0
    slots_per_node: int = 0
    editable: bool = False

    def tx_by_index(self):
        return {c.index: c for c in self.tx}

    def rx_by_index(self):
        return {c.index: c for c in self.rx}


def _raw_out(index: int) -> ChannelDef:
    return ChannelDef(index, f"slot{index}", RAW_OUT, group="raw", min=-32768, max=32767)


def _raw_in(index: int) -> ChannelDef:
    return ChannelDef(index, f"slot{index}", RAW_IN, group="raw", min=-32768, max=32767)


def _fill_remaining(defs: List[ChannelDef], factory) -> List[ChannelDef]:
    used = {c.index for c in defs}
    out = list(defs)
    for i in range(SLOT_COUNT):
        if i not in used:
            out.append(factory(i))
    out.sort(key=lambda c: c.index)
    return out


def make_generic_raw_profile() -> DeviceProfile:
    """CANノード分配を意識せず、ホストのUARTフレーム24スロットをそのまま扱う汎用プロファイル。"""
    tx = [_raw_out(i) for i in range(SLOT_COUNT)]
    rx = [_raw_in(i) for i in range(SLOT_COUNT)]
    return DeviceProfile(
        key="generic_raw",
        name="汎用 Raw (24スロット、CAN分配なし)",
        description="ホストが送受信するUARTフレーム24スロットをそのまま直接編集/表示する。",
        tx=tx,
        rx=rx,
    )


def make_can_host_profile(
    key: str = "xiao_smd_can_host",
    name: str = "XIAO ESP32S3 SMD (CAN Host)",
    node_count: int = DEFAULT_CAN_NODE_COUNT,
    slots_per_node: int = DEFAULT_CAN_SLOTS_PER_NODE,
    motor_max: int = 255,
    servo_min_deg: int = 0,
    servo_max_deg: int = 270,
) -> DeviceProfile:
    """xiao_esp32_s3_smd_serial_bridge MODE_CAN_HOST 用プロファイル。

    24スロットを CAN バス上の最大 node_count ノードへ slots_per_node ずつ分配する。
    各ノード (既定 5スロット):
      指令: MD1, MD2, MD3, MD4, SERVO1
      帰還: SW1, SW2, SW3, ENC1, ENC2
    CAN_ID は 101,102,103,104 のようにノード番号 (1-origin) を下2桁に持つ。
    """
    tx: List[ChannelDef] = []
    rx: List[ChannelDef] = []

    for node in range(node_count):
        base = node * slots_per_node
        node_no = node + 1
        can_id = 100 + node_no
        group_cmd = f"ノード{node_no} (CAN_ID={can_id}) 指令"
        group_fb = f"ノード{node_no} (CAN_ID={can_id}) 帰還"

        motor_slots = min(4, slots_per_node)
        for m in range(motor_slots):
            tx.append(ChannelDef(base + m, f"N{node_no} MD{m + 1}", MOTOR,
                                  group=group_cmd, min=-motor_max, max=motor_max, unit="pwm"))
        if slots_per_node > 4:
            tx.append(ChannelDef(base + 4, f"N{node_no} SERVO1", SERVO,
                                  group=group_cmd, min=servo_min_deg, max=servo_max_deg, unit="deg"))

        sw_slots = min(3, slots_per_node)
        for s in range(sw_slots):
            rx.append(ChannelDef(base + s, f"N{node_no} SW{s + 1}", DIGITAL_IN, group=group_fb))
        if slots_per_node > 3:
            rx.append(ChannelDef(base + 3, f"N{node_no} ENC1", COUNTER, group=group_fb, unit="count"))
        if slots_per_node > 4:
            rx.append(ChannelDef(base + 4, f"N{node_no} ENC2", COUNTER, group=group_fb, unit="count"))

    tx = _fill_remaining(tx, _raw_out)
    rx = _fill_remaining(rx, _raw_in)

    return DeviceProfile(
        key=key,
        name=name,
        description=(
            f"MODE_CAN_HOST 用。24スロットを CAN バス上の最大{node_count}ノードへ"
            f"{slots_per_node}スロットずつ分配する。各ノードは MD1-4 + SERVO1 (指令) / "
            "SW1-3 + ENC1-2 (帰還) を担当。ノード数・スロット数はプロファイル編集で変更可能。"
        ),
        tx=tx,
        rx=rx,
        node_count=node_count,
        slots_per_node=slots_per_node,
    )


def _build_builtin_profiles() -> "dict[str, DeviceProfile]":
    profiles: List[DeviceProfile] = [
        make_can_host_profile(),
        make_generic_raw_profile(),
    ]
    return {p.key: p for p in profiles}


BUILTIN_PROFILES = _build_builtin_profiles()

DEFAULT_PROFILE_KEY = "xiao_smd_can_host"


# ================= カスタムプロファイルの保存/読込 =================
#
# GUI の「プロファイル編集」で作成/変更したプロファイルは
# ~/.config/ros2can/profiles/*.json に保存し、次回起動時に読み込む。
# CAN_NODE_COUNT / CAN_SLOTS_PER_NODE をファームウェア側で変更した場合に利用する。

import json
import os


def custom_profiles_dir() -> str:
    base = os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config"))
    path = os.path.join(base, "ros2can", "profiles")
    os.makedirs(path, exist_ok=True)
    return path


def _channel_to_dict(c: ChannelDef) -> dict:
    return {
        "index": c.index, "label": c.label, "kind": c.kind, "group": c.group,
        "min": c.min, "max": c.max, "step": c.step, "unit": c.unit,
        "scale": c.scale, "decimals": c.decimals, "options": c.options, "note": c.note,
    }


def _channel_from_dict(d: dict) -> ChannelDef:
    options = d.get("options")
    if options is not None:
        options = [tuple(o) for o in options]
    return ChannelDef(
        index=d["index"], label=d.get("label", f"slot{d['index']}"),
        kind=d.get("kind", RAW_OUT), group=d.get("group", ""),
        min=d.get("min", -32768), max=d.get("max", 32767), step=d.get("step", 1),
        unit=d.get("unit", ""), scale=d.get("scale", 1.0), decimals=d.get("decimals", 0),
        options=options, note=d.get("note", ""),
    )


def profile_to_dict(p: DeviceProfile) -> dict:
    return {
        "key": p.key, "name": p.name, "description": p.description,
        "node_count": p.node_count, "slots_per_node": p.slots_per_node,
        "tx": [_channel_to_dict(c) for c in p.tx],
        "rx": [_channel_to_dict(c) for c in p.rx],
    }


def profile_from_dict(d: dict) -> DeviceProfile:
    return DeviceProfile(
        key=d["key"], name=d.get("name", d["key"]), description=d.get("description", ""),
        tx=[_channel_from_dict(c) for c in d.get("tx", [])],
        rx=[_channel_from_dict(c) for c in d.get("rx", [])],
        node_count=d.get("node_count", 0), slots_per_node=d.get("slots_per_node", 0),
        editable=True,
    )


def save_custom_profile(p: DeviceProfile) -> str:
    path = os.path.join(custom_profiles_dir(), f"{p.key}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(profile_to_dict(p), f, ensure_ascii=False, indent=2)
    return path


def delete_custom_profile(key: str) -> None:
    path = os.path.join(custom_profiles_dir(), f"{key}.json")
    if os.path.exists(path):
        os.remove(path)


def load_custom_profiles() -> "dict[str, DeviceProfile]":
    result = {}
    directory = custom_profiles_dir()
    for fname in sorted(os.listdir(directory)):
        if not fname.endswith(".json"):
            continue
        try:
            with open(os.path.join(directory, fname), "r", encoding="utf-8") as f:
                d = json.load(f)
            p = profile_from_dict(d)
            result[p.key] = p
        except (OSError, ValueError, KeyError):
            continue
    return result


def all_profiles() -> "dict[str, DeviceProfile]":
    merged = dict(BUILTIN_PROFILES)
    merged.update(load_custom_profiles())
    return merged


def unique_custom_key(base_key: str, existing: "dict[str, DeviceProfile]") -> str:
    candidate = f"{base_key}_custom"
    n = 2
    while candidate in existing:
        candidate = f"{base_key}_custom{n}"
        n += 1
    return candidate
