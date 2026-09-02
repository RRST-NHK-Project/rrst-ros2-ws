#!/usr/bin/env python3
"""soki_sim: 吸着ハンド(3吸着パッド・展開サーボ・ピッチサーボ・ダイヤフラムポンプ)
制御ノード。

tip_link(tip_theta_jointの先)に付く手先機構(note/claude.txt「##ハンド」参照):
  - 吸着パッド展開サーボ: 収納/展開の2姿勢切替。展開でワーク(最大3個)を吸着し、
    収納で搬送姿勢に戻す
  - ワークピッチ変更サーボ: 保持姿勢/投入姿勢の2姿勢切替。シューティングボックスへ
    縦向きに投入するため、保持したワークの向きを変える
  - ダイヤフラムポンプ: 3吸着パッドの真空源。吸着パッド展開(deploy)と連動して
    ON(pump_duty_percent、既定50%)、収納(retract)と連動してOFF(duty=0)にする
    (ユーザー判断、2026-09-03)

いずれもxiao-esp32-s3_can2io MODE_CAN_HOST配下の汎用IOノード(SERVOx3/MDx2、
ros2can/ros2can/device_profiles.py参照)経由でCAN指令を送る。CubeMars/RoboMasの
MITモード(trajectory_follower_node/real_joint_bridge_node)とは別のチャンネル
体系(角度[deg]の直接指令、PWM+DIRのMDデューティ)のため、独立したノードに
分離している。

サーボはSERVOn(n=1-3、config.hppのMULTIn=1時のみ有効。SWnとピン共有のため、
同じノードの当該local_indexをスイッチ入力として使っている場合は競合する)、
ポンプはMDn(n=1-2、ENCn_MD=1時のみ有効。ENCnとピン共有)。
serial_tx_{device_id}(Int16MultiArray、24スロット)のスロット
(node_index*can_slots_per_node + local_index)に、サーボは角度[deg]の生値、
MDは符号=方向・絶対値=PWMデューティの生値をセットして送信する
(ros2can/ros2can/device_profiles.py _append_generic_io_node_channels参照)。
同じdevice_idを複数チャンネルで共有する場合に備え、device_idごとに24スロットの
バッファを保持し、対象スロットのみ更新して送信する(trajectory_follower_nodeの
CubeMars/RoboMas実装と同じパターン)。

device_id=0(既定)はその出力が未配線・無効であることを意味する(他ノードの
limit_switch_device_id等と同じ規約)。実機配線が判明したら
note/can_mapping.txt「## ハンド」節と、soki_sim/config/hand.yamlのパラメータを
実際の値に更新すること。

サービス(std_srvs/Trigger)でGUI等から明示的に呼び出す設計
(homing_nodeと同様、自動起動はしない):
  /hand_deploy_pads      : 吸着パッド展開角度へ + ポンプduty_percentへ(吸着ON)
  /hand_retract_pads     : 吸着パッド収納角度へ + ポンプduty=0(吸着OFF)
  /hand_set_pitch_hold   : ピッチサーボを保持姿勢へ
  /hand_set_pitch_insert : ピッチサーボを投入姿勢へ

ros2can側の前提: 対象CAN_HOSTデバイスのtopic_passthroughをGUIでONにしておかないと
本ノードの/serial_tx_[id]への指令が実機へ反映されない(homing_node等と同様)。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import Trigger

SLOT_COUNT = 24


class HandNode(Node):

    def __init__(self):
        super().__init__('hand_node')

        self.declare_parameter('can_slots_per_node', 5)
        self.slots_per_node_ = int(self.get_parameter('can_slots_per_node').value)

        # ---- 吸着パッド展開サーボ (SERVOn、角度[deg]) ----
        self.declare_parameter('deploy_servo_device_id', 0)  # 0=未配線・無効
        self.declare_parameter('deploy_servo_node_index', 0)
        self.declare_parameter('deploy_servo_local_index', 0)  # SERVO1=0/SERVO2=1/SERVO3=2
        self.declare_parameter('deploy_servo_retracted_deg', 0)
        self.declare_parameter('deploy_servo_deployed_deg', 90)

        # ---- ワークピッチ変更サーボ (SERVOn、角度[deg]) ----
        self.declare_parameter('pitch_servo_device_id', 0)
        self.declare_parameter('pitch_servo_node_index', 0)
        self.declare_parameter('pitch_servo_local_index', 1)
        self.declare_parameter('pitch_servo_hold_deg', 0)
        self.declare_parameter('pitch_servo_insert_deg', 90)

        # ---- ダイヤフラムポンプ (MDn、符号=方向・絶対値=PWMデューティ) ----
        self.declare_parameter('pump_device_id', 0)
        self.declare_parameter('pump_node_index', 0)
        self.declare_parameter('pump_local_index', 0)  # MD1=0/MD2=1
        self.declare_parameter('pump_duty_percent', 50.0)
        # config.hppのMD_PWM_RESOLUTION(既定8bit)から算出される最大デューティ値。
        # ros2can/ros2can/device_profiles.pyのDEFAULT_MD_PWM_MAXと一致させること。
        self.declare_parameter('pump_md_pwm_max', 255)

        self._deploy = self._make_channel_cfg('deploy_servo')
        self._pitch = self._make_channel_cfg('pitch_servo')
        self._pump = self._make_channel_cfg('pump')

        self.device_buffers_ = {}
        self.device_publishers_ = {}
        for cfg in (self._deploy, self._pitch, self._pump):
            self._ensure_publisher(cfg['device_id'])

        self.create_service(Trigger, 'hand_deploy_pads', self._on_deploy_pads)
        self.create_service(Trigger, 'hand_retract_pads', self._on_retract_pads)
        self.create_service(Trigger, 'hand_set_pitch_hold', self._on_set_pitch_hold)
        self.create_service(Trigger, 'hand_set_pitch_insert', self._on_set_pitch_insert)

        self.get_logger().info(
            'hand_node started: '
            f'deploy_servo(device_id={self._deploy["device_id"]}), '
            f'pitch_servo(device_id={self._pitch["device_id"]}), '
            f'pump(device_id={self._pump["device_id"]}, '
            f'duty={self.get_parameter("pump_duty_percent").value}%)')

    def _make_channel_cfg(self, prefix):
        return {
            'device_id': int(self.get_parameter(f'{prefix}_device_id').value),
            'node_index': int(self.get_parameter(f'{prefix}_node_index').value),
            'local_index': int(self.get_parameter(f'{prefix}_local_index').value),
        }

    def _ensure_publisher(self, device_id):
        if device_id == 0 or device_id in self.device_publishers_:
            return
        self.device_buffers_[device_id] = [0] * SLOT_COUNT
        self.device_publishers_[device_id] = self.create_publisher(
            Int16MultiArray, f'serial_tx_{device_id}', 10)

    def _slot(self, cfg):
        return cfg['node_index'] * self.slots_per_node_ + cfg['local_index']

    def _send(self, cfg, raw_value):
        """cfgの示すスロットへraw_value(サーボなら角度[deg]、MDなら符号付き
        デューティ)をセットし、そのdevice_idの24スロット全体を再送する
        (他スロットは前回セットした値を保持したまま送る)。device_id=0
        (未配線)なら何もせずFalseを返す。"""
        device_id = cfg['device_id']
        if device_id == 0:
            self.get_logger().warning(
                f'hand_node: device_id未設定のため送信をスキップしました(値={raw_value})')
            return False
        buf = self.device_buffers_[device_id]
        buf[self._slot(cfg)] = int(raw_value)
        msg = Int16MultiArray()
        msg.data = list(buf)
        self.device_publishers_[device_id].publish(msg)
        return True

    def _on_deploy_pads(self, request, response):
        deg = int(self.get_parameter('deploy_servo_deployed_deg').value)
        ok_servo = self._send(self._deploy, deg)
        duty_percent = float(self.get_parameter('pump_duty_percent').value)
        pwm_max = int(self.get_parameter('pump_md_pwm_max').value)
        duty_raw = round(pwm_max * duty_percent / 100.0)
        ok_pump = self._send(self._pump, duty_raw)
        response.success = ok_servo and ok_pump
        response.message = (
            f'吸着パッド展開({deg}deg) + ポンプON({duty_percent:.0f}%)' if response.success
            else 'device_id未設定のため一部/全部を送信できませんでした')
        return response

    def _on_retract_pads(self, request, response):
        deg = int(self.get_parameter('deploy_servo_retracted_deg').value)
        ok_servo = self._send(self._deploy, deg)
        ok_pump = self._send(self._pump, 0)
        response.success = ok_servo and ok_pump
        response.message = (
            f'吸着パッド収納({deg}deg) + ポンプOFF' if response.success
            else 'device_id未設定のため一部/全部を送信できませんでした')
        return response

    def _on_set_pitch_hold(self, request, response):
        deg = int(self.get_parameter('pitch_servo_hold_deg').value)
        response.success = self._send(self._pitch, deg)
        response.message = f'ピッチ: 保持姿勢({deg}deg)' if response.success else 'device_id未設定です'
        return response

    def _on_set_pitch_insert(self, request, response):
        deg = int(self.get_parameter('pitch_servo_insert_deg').value)
        response.success = self._send(self._pitch, deg)
        response.message = f'ピッチ: 投入姿勢({deg}deg)' if response.success else 'device_id未設定です'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
