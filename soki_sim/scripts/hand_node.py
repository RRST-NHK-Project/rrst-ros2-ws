#!/usr/bin/env python3
"""soki_sim: 吸着ハンド(3吸着パッド・展開サーボ・ピッチサーボ・ダイヤフラムポンプ)
制御ノード。

tip_link(tip_theta_jointの先)に付く手先機構(note/claude.txt「##ハンド」参照):
  - 吸着パッド3個: 同一行に並んだワーク3個(間隔0.200m、command_gui_node.pyの
    WORK_COL_PITCHと一致)へ同時に降ろして吸着する展開(spread)位置と、吸着後に
    中央へ集める収納(gathered、3パッド上面中心が正三角形になる)位置の2状態
  - 吸着パッド展開/収集サーボ: 展開(spread、ワーク回収時)/収納(gathered、
    搬送時)の2姿勢切替
  - ワークピッチ変更サーボ: 保持姿勢/投入姿勢の2姿勢切替。シューティングボックスへ
    縦向きに投入するため、保持したワークの向きを変える
  - ダイヤフラムポンプ: 3吸着パッドの真空源

展開/収集サーボとポンプは独立した別々の指令(2026-09-03、ユーザー指摘で修正:
当初はdeploy=展開+ポンプON、retract=収納+ポンプOFFと連動させていたが、収納は
「ワークを保持したまま中央へ集める」動作でありポンプを切ってはいけないため、
サーボとポンプを別サービスに分離した)。ポンプのON/OFFタイミングは呼び出し側
(GUI/上位シーケンス)が判断する。

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
  /hand_spread_pads      : 吸着パッド展開角度へ(spread、ワーク回収姿勢)
  /hand_gather_pads      : 吸着パッド収納角度へ(gathered、正三角形に集約)
  /hand_pump_on          : ポンプduty_percentへ(吸着ON)
  /hand_pump_off         : ポンプduty=0へ(吸着OFF)
  /hand_set_pitch_hold   : ピッチサーボを保持姿勢へ
  /hand_set_pitch_insert : ピッチサーボを投入姿勢へ

ros2can側の前提: 対象CAN_HOSTデバイスのtopic_passthroughをGUIでONにしておかないと
本ノードの/serial_tx_[id]への指令が実機へ反映されない(homing_node等と同様)。

上記4サービスはCAN送信(device_id=0なら未配線としてスキップ)とは別に、
soki_sim.urdf.xacroのsuction_pad_1/2/3_joint・hand_pitch_joint(2026-09-03追加の
簡易形状)の位置を/mixed_joint_states(motor_mixer_node/trajectory_follower_node
と同じ、joint_state_publisher(_gui)のsource_list集約トピック)へpublishする。
実機配線の有無に関わらず常に送るため、配線前でもGUI操作でRVizの見た目が
連動する(display.launch.py参照)。起動直後にも収納(gathered)/保持姿勢で
一度publishし、未配線状態でもRVizが「関節状態を受信していない」警告に
ならないようにしている。

suction_pad_1/2/3_jointはprismatic関節で、joint値0=spread(展開、同一行の
ワーク間隔で並ぶ)、joint値=HAND_PAD_TRAVELの上限=gathered(収納、中央パッドを
含む正三角形に集まる。中央のsuction_pad_2は動かない)を表す
(soki_sim.urdf.xacroのhand_pad_side_travelと一致させること)。
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import Trigger

SLOT_COUNT = 24
HAND_PITCH_JOINT = 'hand_pitch_joint'
# soki_sim.urdf.xacroのhand_pad_side_travelと一致させること。
# joint値0=spread(展開)、joint値=この上限値=gathered(収納、正三角形)。
# suction_pad_2(中央)は動かない(2026-09-03修正: 中央は固定、両端2枚だけが
# 動いて中央パッドを含む正三角形を作る構成に訂正)。
HAND_PAD_TRAVEL = {
    'suction_pad_1_joint': 0.174356,
    'suction_pad_2_joint': 0.0,
    'suction_pad_3_joint': 0.174356,
}


class HandNode(Node):

    def __init__(self):
        super().__init__('hand_node')

        self.declare_parameter('can_slots_per_node', 5)
        self.slots_per_node_ = int(self.get_parameter('can_slots_per_node').value)

        # SERVOn(角度[deg])に送る値の許容範囲。実機のホビーサーボは負の角度を
        # 受け付けない/暴走することがあるため、deploy/pitch両サーボへ送信する
        # 直前に必ずこの範囲へクランプする(2026-09-03追加。pitch_servo_offset_deg
        # を導入した結果、hold_deg/insert_deg+offsetが負値になり実機で問題に
        # なったための対処)。ros2can/ros2can/device_profiles.pyのSERVO既定範囲
        # (0〜270deg)と合わせてある。
        self.declare_parameter('servo_min_deg', 0)
        self.declare_parameter('servo_max_deg', 270)

        # ---- 吸着パッド展開サーボ (SERVOn、角度[deg]) ----
        self.declare_parameter('deploy_servo_device_id', 0)  # 0=未配線・無効
        self.declare_parameter('deploy_servo_node_index', 0)
        self.declare_parameter('deploy_servo_local_index', 0)  # SERVO1=0/SERVO2=1/SERVO3=2
        self.declare_parameter('deploy_servo_retracted_deg', 0)
        self.declare_parameter('deploy_servo_deployed_deg', 90)
        # 実機のサーボ取付角度と論理角度(retracted_deg/deployed_deg)のズレを
        # 補正するオフセット。CAN送信にのみ加算する(RViz表示(spread/gathered
        # のパッド位置)には影響しない。pitch_servo_offset_degと同じ設計、
        # 2026-09-03追加)。
        self.declare_parameter('deploy_servo_offset_deg', 0.0)

        # ---- ワークピッチ変更サーボ (SERVOn、角度[deg]) ----
        self.declare_parameter('pitch_servo_device_id', 0)
        self.declare_parameter('pitch_servo_node_index', 0)
        self.declare_parameter('pitch_servo_local_index', 1)
        self.declare_parameter('pitch_servo_hold_deg', 0)
        self.declare_parameter('pitch_servo_insert_deg', 90)
        # 実機のサーボ取付角度と論理角度(hold=0deg/insert=90deg)のズレを補正する
        # オフセット。CAN送信にのみ加算する(RViz表示には影響しない。2026-09-03
        # 修正: 当初はRViz表示にも加算していたが、クランプ発生時にRViz上の
        # 見た目まで意図しない値になり「動作イメージとリンクしていた表示が
        # 崩れる」問題があったため分離した。「ピッチ軸の原点の場所を
        # 変えたい」との要望に対応)。
        self.declare_parameter('pitch_servo_offset_deg', 0.0)

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

        # RViz表示用(soki_sim.urdf.xacroのhand_deploy_joint/hand_pitch_joint、
        # モジュールdocstring参照)。実機配線(device_id)の有無に関わらず送る。
        self.joint_pub_ = self.create_publisher(JointState, 'mixed_joint_states', 10)

        self.create_service(Trigger, 'hand_spread_pads', self._on_spread_pads)
        self.create_service(Trigger, 'hand_gather_pads', self._on_gather_pads)
        self.create_service(Trigger, 'hand_pump_on', self._on_pump_on)
        self.create_service(Trigger, 'hand_pump_off', self._on_pump_off)
        self.create_service(Trigger, 'hand_set_pitch_hold', self._on_set_pitch_hold)
        self.create_service(Trigger, 'hand_set_pitch_insert', self._on_set_pitch_insert)

        # 起動直後の姿勢(収納/保持)を一度publishしておく。CANへは送らない
        # (device_id未配線でも安全なように、起動時は実機へ何も指令しない設計を維持)。
        # RViz表示はpitch_servo_offset_degを加算しない論理角度のまま(2026-09-03
        # 修正: sim表示と実機送信値を分離した経緯はHAND_PITCH_JOINT関連の
        # コメント参照)。
        self._publish_pad_states(gathered=True)
        self._publish_joint_state(
            HAND_PITCH_JOINT, int(self.get_parameter('pitch_servo_hold_deg').value))

        self.get_logger().info(
            'hand_node started: '
            f'deploy_servo(device_id={self._deploy["device_id"]}), '
            f'pitch_servo(device_id={self._pitch["device_id"]}), '
            f'pump(device_id={self._pump["device_id"]}, '
            f'duty={self.get_parameter("pump_duty_percent").value}%)')

    def _publish_joint_state(self, joint_name, deg):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_name]
        msg.position = [math.radians(deg)]
        self.joint_pub_.publish(msg)

    def _publish_pad_states(self, gathered):
        """3吸着パッドのRViz表示位置を更新する。gathered=Falseならspread
        (展開、同一行のワーク間隔)、Trueならgathered(収納、正三角形)。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(HAND_PAD_TRAVEL.keys())
        msg.position = [HAND_PAD_TRAVEL[name] if gathered else 0.0 for name in msg.name]
        self.joint_pub_.publish(msg)

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

    def _clamp_servo_deg(self, deg):
        """SERVOnへ送る角度[deg]をservo_min_deg〜servo_max_degへクランプする。
        実機のホビーサーボは負の角度等、範囲外の値を受け付けない/暴走する
        ことがあるため、CAN送信直前の値にのみ適用する(RViz表示には使わない。
        2026-09-03追加)。クランプが発生した場合はログ警告を出す。"""
        lo = int(self.get_parameter('servo_min_deg').value)
        hi = int(self.get_parameter('servo_max_deg').value)
        clamped = max(lo, min(hi, deg))
        if clamped != deg:
            self.get_logger().warning(
                f'hand_node: 角度{deg}degはサーボ可動範囲[{lo},{hi}]外のため'
                f'{clamped}degにクランプしました')
        return clamped

    def _deploy_offset_deg(self):
        return float(self.get_parameter('deploy_servo_offset_deg').value)

    def _set_deploy(self, sim_deg, gathered, response, label):
        """RViz表示(gathered、sim_degの値自体は使わずspread/gathered状態のみ
        反映)と、実際にCANへ送る角度(sim_deg + deploy_servo_offset_deg、
        クランプ済み)を分けて扱う(_set_pitchと同じ設計、2026-09-03追加。
        「収納・展開サーボにも同様の機能を」との要望に対応)。"""
        can_deg = self._clamp_servo_deg(round(sim_deg + self._deploy_offset_deg()))
        response.success = self._send(self._deploy, can_deg)
        self._publish_pad_states(gathered=gathered)
        response.message = (
            f'吸着パッド{label}(sim={sim_deg}deg, 実機送信={can_deg}deg)' if response.success
            else f'吸着パッド{label}(sim={sim_deg}deg、表示のみ): device_id未設定のためCAN送信できませんでした')
        return response

    def _on_spread_pads(self, request, response):
        sim_deg = int(self.get_parameter('deploy_servo_deployed_deg').value)
        return self._set_deploy(sim_deg, False, response, '展開')

    def _on_gather_pads(self, request, response):
        sim_deg = int(self.get_parameter('deploy_servo_retracted_deg').value)
        return self._set_deploy(sim_deg, True, response, '収納')

    def _on_pump_on(self, request, response):
        duty_percent = float(self.get_parameter('pump_duty_percent').value)
        pwm_max = int(self.get_parameter('pump_md_pwm_max').value)
        duty_raw = round(pwm_max * duty_percent / 100.0)
        response.success = self._send(self._pump, duty_raw)
        response.message = (
            f'ポンプON({duty_percent:.0f}%)' if response.success
            else 'ポンプON: device_id未設定のためCAN送信できませんでした')
        return response

    def _on_pump_off(self, request, response):
        response.success = self._send(self._pump, 0)
        response.message = 'ポンプOFF' if response.success else 'ポンプOFF: device_id未設定のためCAN送信できませんでした'
        return response

    def _pitch_offset_deg(self):
        return float(self.get_parameter('pitch_servo_offset_deg').value)

    def _set_pitch(self, sim_deg, response, label):
        """RViz表示(sim_deg、pitch_servo_hold_deg/insert_degそのまま)と、
        実際にCANへ送る角度(sim_deg + pitch_servo_offset_deg、servoの可動範囲へ
        クランプ済み)を分けて扱う(2026-09-03修正: 当初はoffset加算後の値を
        表示にも使っていたため、実機の可動範囲に合わせてクランプすると
        RViz上の見た目まで意図しない値になり「動作イメージとリンクしていた
        表示が崩れる」問題があった。sim表示は常にオフセット・クランプの
        影響を受けない論理角度のままにする)。"""
        can_deg = self._clamp_servo_deg(round(sim_deg + self._pitch_offset_deg()))
        response.success = self._send(self._pitch, can_deg)
        self._publish_joint_state(HAND_PITCH_JOINT, sim_deg)
        response.message = (
            f'ピッチ: {label}(sim={sim_deg}deg, 実機送信={can_deg}deg)' if response.success
            else f'ピッチ: {label}(sim={sim_deg}deg、表示のみ): device_id未設定です')
        return response

    def _on_set_pitch_hold(self, request, response):
        sim_deg = int(self.get_parameter('pitch_servo_hold_deg').value)
        return self._set_pitch(sim_deg, response, '保持姿勢')

    def _on_set_pitch_insert(self, request, response):
        sim_deg = int(self.get_parameter('pitch_servo_insert_deg').value)
        return self._set_pitch(sim_deg, response, '投入姿勢')


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
