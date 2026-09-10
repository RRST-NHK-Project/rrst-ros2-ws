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
  /hand_pump_on          : ポンプduty_percentへ(吸着ON)。真空破壊リレーはOFF(閉)
  /hand_pump_off         : ポンプduty=0へ(吸着OFF)。真空破壊リレーをON(開)にして
                           ワークを離す(2026-09-07新規、MD2のDIRピンをリレー駆動
                           信号として流用。独立サービスは設けず連動のみ)
  /hand_set_pitch_hold   : ピッチサーボを保持姿勢へ
  /hand_set_pitch_insert : ピッチサーボを投入姿勢へ

収納/展開・保持/投入の4状態それぞれについて、実機へ送るCAN角度を
`*_can_deg_override`(bool)がTrueの間`*_can_deg`(float)に固定できる
(2026-09-03新規、_resolve_can_deg参照)。Falseなら従来通り論理角度
(*_deg)+オフセット(*_offset_deg)を使う。いずれの経路でもクランプは行わない
(以前はservo_min_deg/servo_max_degへクランプしていたが、ユーザー指摘
「オフセットをクランプしない。実機との相違があるため手動で固定角度を設定する」
により撤廃し、代わりにこの手動オーバーライドを追加した。実機のホビーサーボが
受け付けない/暴走する値を送らないよう、offset・オーバーライド値とも実機で
安全な範囲に手動で追い込んで運用すること)。

ポンプの現在ON/OFF状態は`hand_pump_state`(std_msgs/Bool)、吸着パッドの展開状態は
`hand_pads_spread`、ワークピッチの投入姿勢状態は`hand_pitch_insert`として
publishする(ポンプは2026-09-03、残り2つは2026-09-10追加。いずれも
transient_local(latched))。後者2つはjoy_teleop_nodeのL3/R3トグルが
「今どちらの姿勢か」を知るために使う。GUIハンドパネルの「ポンプON/OFF」ボタンとjoy_teleop_nodeの
PSコン丸ボタン(トグル)の両方から独立に操作できるようにするため、状態の真値は
hand_node側に一元化し、joy_teleop_node側ではローカルに状態を推測しない。

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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import Trigger

SLOT_COUNT = 24
# 状態表示灯の点滅トグル間隔[ms]。command_gui_node.pyのLedIndicatorWidgetと
# 一致させること(sim表示と実機LEDの見た目を揃えるため、2026-09-08追加)。
LED_BLINK_FAST_PERIOD_MS = 150
LED_BLINK_SLOW_PERIOD_MS = 500
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

        # ---- 吸着パッド展開サーボ (SERVOn、角度[deg]) ----
        self.declare_parameter('deploy_servo_device_id', 0)  # 0=未配線・無効
        self.declare_parameter('deploy_servo_node_index', 0)
        self.declare_parameter('deploy_servo_local_index', 0)  # SERVO1=0/SERVO2=1/SERVO3=2
        self.declare_parameter('deploy_servo_retracted_deg', 0)
        self.declare_parameter('deploy_servo_deployed_deg', 90)
        # 実機のサーボ取付角度と論理角度(retracted_deg/deployed_deg)のズレを
        # 補正するオフセット。CAN送信にのみ加算する(RViz表示(spread/gathered
        # のパッド位置)には影響しない。pitch_servo_offset_degと同じ設計、
        # 2026-09-03追加)。クランプは行わない(2026-09-03、ユーザー指摘:
        # 「オフセットをクランプしない」。実機のホビーサーボは負角度等を
        # 受け付けない/暴走することがあるため、offset自体を実機で安全な範囲に
        # 手動で追い込んで運用すること)。
        self.declare_parameter('deploy_servo_offset_deg', 0.0)
        # 収納/展開それぞれについて、sim_deg+offsetの計算を使わず実機送信角度を
        # 直接固定したい場合の手動オーバーライド(2026-09-03新規、ユーザー指摘:
        # 「実機との相違があるため手動で固定角度を設定する」)。
        # *_can_deg_overrideがTrueの間は*_can_degをそのままCAN送信する。
        self.declare_parameter('deploy_servo_retracted_can_deg_override', False)
        self.declare_parameter('deploy_servo_retracted_can_deg', 0.0)
        self.declare_parameter('deploy_servo_deployed_can_deg_override', False)
        self.declare_parameter('deploy_servo_deployed_can_deg', 0.0)

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
        # 変えたい」との要望に対応)。クランプは行わない(deploy_servo_offset_deg
        # と同じ理由、2026-09-03)。
        self.declare_parameter('pitch_servo_offset_deg', 0.0)
        # 保持/投入それぞれの実機送信角度を直接固定する手動オーバーライド
        # (deploy_servo_*_can_degと同じ設計、2026-09-03新規)。
        self.declare_parameter('pitch_servo_hold_can_deg_override', False)
        self.declare_parameter('pitch_servo_hold_can_deg', 0.0)
        self.declare_parameter('pitch_servo_insert_can_deg_override', False)
        self.declare_parameter('pitch_servo_insert_can_deg', 0.0)

        # ---- ダイヤフラムポンプ (MDn、符号=方向・絶対値=PWMデューティ。SERVO1-3=
        # local0-2なので、汎用IOノード1台分ではMD1=local3/MD2=local4になる) ----
        self.declare_parameter('pump_device_id', 0)
        self.declare_parameter('pump_node_index', 0)
        self.declare_parameter('pump_local_index', 3)  # MD1=3/MD2=4
        self.declare_parameter('pump_duty_percent', 50.0)
        # config.hppのMD_PWM_RESOLUTION(既定8bit)から算出される最大デューティ値。
        # ros2can/ros2can/device_profiles.pyのDEFAULT_MD_PWM_MAXと一致させること。
        self.declare_parameter('pump_md_pwm_max', 255)

        # ---- 真空破壊リレー (MD2のDIRピンをリレー駆動信号として流用。2026-09-07
        # 新規: ワークを離す際の真空破壊にリレーを使い、そのリレーをMDの方向ピンで
        # 代用する。DIRピンは`digitalWrite(MDnD, raw>0 ? HIGH : LOW)`のようにduty量
        # とは無関係に符号だけで決まる(ros2can/firmware/xiao-esp32-s3_can2io/src/
        # pin_ctrl_task.cpp IO_MD_Output参照、PWM出力自体は未配線なので無視される)。
        # 独立サービスにはせず、ポンプON中はリレーOFF(閉、真空保持)・ポンプOFF時は
        # リレーON(開、真空破壊)という連動で_on_pump_on/_on_pump_offから駆動する) ----
        self.declare_parameter('vacuum_release_device_id', 0)
        self.declare_parameter('vacuum_release_node_index', 0)
        self.declare_parameter('vacuum_release_local_index', 4)  # MD2
        self.declare_parameter('vacuum_release_duty_percent', 50.0)

        # ---- 状態表示灯(黄色/赤色LED、SERVOn=デジタル出力。2026-09-08追加) ----
        # CAN_HOST device_id=101のMULTI1/MULTI2にそれぞれ接続(ファームウェアは
        # soki_host_led_101、MULTI1/MULTI2=2でSERVOnピンをデジタル出力へ切替済み、
        # CanIoRxData[local_index]の非ゼロでHIGH)。pump/vacuum_releaseと同じ
        # device_id=101・node_index=0(汎用IOノードの自ノード分)を共有するため、
        # 送信は本ノード(device_id=101のCAN送信元)にまとめる必要がある
        # (note/note_soki/can_mapping.txt「## 状態表示灯」参照)。状態自体の
        # 判定ロジックはcommand_gui_node.py(_update_status_leds)に一元化してあり、
        # 本ノードはled_yellow_state/led_red_state(std_msgs/String、値は
        # LedIndicatorWidget.STATE_*と同じ'off'/'on'/'blink_fast'/'blink_slow')を
        # 購読して点滅の位相だけを自前で計算する(_update_led_output参照)。
        self.declare_parameter('led_device_id', 0)
        self.declare_parameter('led_node_index', 0)
        self.declare_parameter('led_yellow_local_index', 0)  # SERVO1=MULTI1
        self.declare_parameter('led_red_local_index', 1)      # SERVO2=MULTI2

        self._deploy = self._make_channel_cfg('deploy_servo')
        self._pitch = self._make_channel_cfg('pitch_servo')
        self._pump = self._make_channel_cfg('pump')
        self._vacuum_release = self._make_channel_cfg('vacuum_release')
        self._led = {
            'device_id': int(self.get_parameter('led_device_id').value),
            'node_index': int(self.get_parameter('led_node_index').value),
            'yellow_local_index': int(self.get_parameter('led_yellow_local_index').value),
            'red_local_index': int(self.get_parameter('led_red_local_index').value),
        }

        self.device_buffers_ = {}
        self.device_publishers_ = {}
        for cfg in (self._deploy, self._pitch, self._pump, self._vacuum_release):
            self._ensure_publisher(cfg['device_id'])
        self._ensure_publisher(self._led['device_id'])

        self._led_yellow_state_ = 'off'
        self._led_red_state_ = 'off'
        led_state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'led_yellow_state', self._on_led_yellow_state, led_state_qos)
        self.create_subscription(String, 'led_red_state', self._on_led_red_state, led_state_qos)
        # 点滅のトグルには周期的な再送が要るため、pump/deploy等と違いタイマ駆動にする
        # (他チャンネルはservice呼び出し時のイベント駆動のみ)。
        self._led_blink_timer_ = self.create_timer(0.1, self._update_led_output)

        # RViz表示用(soki_sim.urdf.xacroのhand_deploy_joint/hand_pitch_joint、
        # モジュールdocstring参照)。実機配線(device_id)の有無に関わらず送る。
        self.joint_pub_ = self.create_publisher(JointState, 'mixed_joint_states', 10)

        # ポンプの現在ON/OFF状態(2026-09-03追加)。GUIハンドパネルとPSコン丸ボタン
        # (joy_teleop_node)の両方から独立にポンプを操作できるようにするため、
        # hand_node側が真値を持ちBoolでpublishする(joy_teleop_node側でトグル
        # 判定に使う。自分でローカルに状態を推測させるとGUI操作とズレるため)。
        self._pump_on = False
        # transient_local(いわゆるlatched)にしておくことで、command_gui_node/
        # joy_teleop_nodeがhand_node起動後に立ち上がった場合でも、次のON/OFF操作を
        # 待たずに起動時点のポンプ状態を受け取れるようにする(2026-09-03、
        # 「ポンプON自動化」対応でこの状態を待つ側が増えたため必要になった)。
        pump_state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pump_state_pub_ = self.create_publisher(Bool, 'hand_pump_state', pump_state_qos)

        # 吸着パッド展開・ワークピッチの現在状態(2026-09-10追加、ユーザー指定:
        # 「ハンドのサーボ操作をPSコンのL3/R3へ割り当て」)。joy_teleop_nodeの
        # L3/R3はトグルなので、押した側が「今どちらの姿勢か」を知る必要がある。
        # ポンプ(hand_pump_state)と同じ設計で、状態の真値はhand_node側に一元化し、
        # 購読側ではローカルに推測しないこと: これらのサービスはGUIのハンド
        # パネル・回収/投入シーケンスからも呼ばれるため、ローカル推測だと
        # 必ずズレる。QoSもポンプと同じtransient_local(latched)にして、
        # hand_node起動後に立ち上がったノードでも現在状態を受け取れるようにする。
        hand_state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pads_spread = False   # 起動時は収納(gathered)姿勢
        self._pitch_insert = False  # 起動時は保持(hold)姿勢
        self.pads_spread_pub_ = self.create_publisher(
            Bool, 'hand_pads_spread', hand_state_qos)
        self.pitch_insert_pub_ = self.create_publisher(
            Bool, 'hand_pitch_insert', hand_state_qos)

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
        self._publish_pump_state()
        self._publish_pads_spread_state()
        self._publish_pitch_insert_state()

        self.get_logger().info(
            'hand_node started: '
            f'deploy_servo(device_id={self._deploy["device_id"]}), '
            f'pitch_servo(device_id={self._pitch["device_id"]}), '
            f'pump(device_id={self._pump["device_id"]}, '
            f'duty={self.get_parameter("pump_duty_percent").value}%), '
            f'vacuum_release(device_id={self._vacuum_release["device_id"]}), '
            f'led(device_id={self._led["device_id"]})')

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

    def _on_led_yellow_state(self, msg):
        self._led_yellow_state_ = msg.data

    def _on_led_red_state(self, msg):
        self._led_red_state_ = msg.data

    def _led_lit(self, state):
        """state('off'/'on'/'blink_fast'/'blink_slow')から、現在の瞬間の
        点灯/消灯(bool)を計算する。位相はノード起動時刻からの経過時間を使う
        自由継続方式(command_gui_node.pyのLedIndicatorWidgetと違い、状態が
        変わった瞬間に位相をリセットしない。実機側では見た目の違いは無視できる)。"""
        if state == 'on':
            return True
        if state not in ('blink_fast', 'blink_slow'):
            return False
        period_ms = LED_BLINK_FAST_PERIOD_MS if state == 'blink_fast' else LED_BLINK_SLOW_PERIOD_MS
        phase_ms = int(time.monotonic() * 1000) % (period_ms * 2)
        return phase_ms < period_ms

    def _update_led_output(self):
        """led_yellow_state/led_red_state(購読済み)から現在の点灯/消灯を計算し、
        device_id=101の24スロットバッファへ反映して送信する(pump/vacuum_release
        と同じバッファを共有するため、他スロットは前回値を保持したまま送る、
        _send参照)。led_device_id=0(未配線)なら何もしない。0.1秒周期タイマ
        (_led_blink_timer_)から呼ばれる。"""
        device_id = self._led['device_id']
        if device_id == 0:
            return
        buf = self.device_buffers_[device_id]
        yellow_slot = self._led['node_index'] * self.slots_per_node_ + self._led['yellow_local_index']
        red_slot = self._led['node_index'] * self.slots_per_node_ + self._led['red_local_index']
        buf[yellow_slot] = 1 if self._led_lit(self._led_yellow_state_) else 0
        buf[red_slot] = 1 if self._led_lit(self._led_red_state_) else 0
        msg = Int16MultiArray()
        msg.data = list(buf)
        self.device_publishers_[device_id].publish(msg)

    def _resolve_can_deg(self, sim_deg, offset_deg, override_prefix):
        """実機へ送るCAN角度[deg]を決定する。<override_prefix>_can_deg_overrideが
        Trueなら<override_prefix>_can_degをそのまま使う(実機との相違を手動で
        直接補正するため、2026-09-03新規、ユーザー指摘: 「実機との相違があるため
        手動で固定角度を設定する」)。Falseなら従来通りsim_deg+offset_degを使う。
        いずれもクランプは行わない(2026-09-03、ユーザー指摘: 「オフセットを
        クランプしない」。実機のホビーサーボは負角度等を受け付けない/暴走する
        ことがあるため、offset・オーバーライド値とも実機で安全な範囲に手動で
        追い込んで運用すること)。"""
        if bool(self.get_parameter(f'{override_prefix}_can_deg_override').value):
            return round(float(self.get_parameter(f'{override_prefix}_can_deg').value))
        return round(sim_deg + offset_deg)

    def _deploy_offset_deg(self):
        return float(self.get_parameter('deploy_servo_offset_deg').value)

    def _set_deploy(self, sim_deg, gathered, response, label, override_prefix):
        """RViz表示(gathered、sim_degの値自体は使わずspread/gathered状態のみ
        反映)と、実際にCANへ送る角度(_resolve_can_deg参照)を分けて扱う
        (_set_pitchと同じ設計、2026-09-03追加。「収納・展開サーボにも同様の
        機能を」との要望に対応)。"""
        can_deg = self._resolve_can_deg(sim_deg, self._deploy_offset_deg(), override_prefix)
        response.success = self._send(self._deploy, can_deg)
        self._publish_pad_states(gathered=gathered)
        # CAN送信の成否(device_id未配線か)に関わらず論理状態は更新する
        # (_on_pump_onと同じ理由。配線前でもjoy_teleop_node側のトグル判定が正しく動く)。
        self._pads_spread = not gathered
        self._publish_pads_spread_state()
        response.message = (
            f'吸着パッド{label}(sim={sim_deg}deg, 実機送信={can_deg}deg)' if response.success
            else f'吸着パッド{label}(sim={sim_deg}deg、表示のみ): device_id未設定のためCAN送信できませんでした')
        return response

    def _on_spread_pads(self, request, response):
        sim_deg = int(self.get_parameter('deploy_servo_deployed_deg').value)
        return self._set_deploy(sim_deg, False, response, '展開', 'deploy_servo_deployed')

    def _on_gather_pads(self, request, response):
        sim_deg = int(self.get_parameter('deploy_servo_retracted_deg').value)
        return self._set_deploy(sim_deg, True, response, '収納', 'deploy_servo_retracted')

    def _publish_pump_state(self):
        msg = Bool()
        msg.data = self._pump_on
        self.pump_state_pub_.publish(msg)

    def _publish_pads_spread_state(self):
        msg = Bool()
        msg.data = self._pads_spread
        self.pads_spread_pub_.publish(msg)

    def _publish_pitch_insert_state(self):
        msg = Bool()
        msg.data = self._pitch_insert
        self.pitch_insert_pub_.publish(msg)

    def _set_vacuum_release(self, on):
        """真空破壊リレー(MD2 DIRピン流用)をON/OFFする。ポンプON/OFFと連動して
        呼ばれる専用の内部ヘルパーで、独立したサービスは設けない(_on_pump_on/
        _on_pump_off参照)。DIRピンは符号のみで決まるため、OFF時は0を送ればよい。"""
        if not on:
            self._send(self._vacuum_release, 0)
            return
        duty_percent = float(self.get_parameter('vacuum_release_duty_percent').value)
        pwm_max = int(self.get_parameter('pump_md_pwm_max').value)
        duty_raw = round(pwm_max * duty_percent / 100.0)
        self._send(self._vacuum_release, duty_raw)

    def _on_pump_on(self, request, response):
        duty_percent = float(self.get_parameter('pump_duty_percent').value)
        pwm_max = int(self.get_parameter('pump_md_pwm_max').value)
        duty_raw = round(pwm_max * duty_percent / 100.0)
        response.success = self._send(self._pump, duty_raw)
        # 真空破壊リレーはポンプONの間は閉じておく(真空保持)。
        self._set_vacuum_release(False)
        # RViz表示用publish等と同様、CAN送信の成否(device_id未配線か)に関わらず
        # 論理状態は更新する(配線前でもjoy_teleop_node側のトグル判定が正しく動く
        # ようにするため)。
        self._pump_on = True
        self._publish_pump_state()
        response.message = (
            f'ポンプON({duty_percent:.0f}%)' if response.success
            else 'ポンプON: device_id未設定のためCAN送信できませんでした')
        return response

    def _on_pump_off(self, request, response):
        response.success = self._send(self._pump, 0)
        # 真空破壊リレーをONにして真空を破壊する(ワークを離す)。
        self._set_vacuum_release(True)
        self._pump_on = False
        self._publish_pump_state()
        response.message = 'ポンプOFF(真空破壊)' if response.success else 'ポンプOFF: device_id未設定のためCAN送信できませんでした'
        return response

    def _pitch_offset_deg(self):
        return float(self.get_parameter('pitch_servo_offset_deg').value)

    def _set_pitch(self, sim_deg, response, label, override_prefix):
        """RViz表示(sim_deg、pitch_servo_hold_deg/insert_degそのまま)と、
        実際にCANへ送る角度(_resolve_can_deg参照)を分けて扱う(2026-09-03修正:
        当初はoffset加算後の値を表示にも使っていたため、実機の可動範囲に
        合わせてクランプすると RViz上の見た目まで意図しない値になり「動作
        イメージとリンクしていた表示が崩れる」問題があった。sim表示は常に
        オフセット・オーバーライドの影響を受けない論理角度のままにする)。"""
        can_deg = self._resolve_can_deg(sim_deg, self._pitch_offset_deg(), override_prefix)
        response.success = self._send(self._pitch, can_deg)
        self._publish_joint_state(HAND_PITCH_JOINT, sim_deg)
        self._pitch_insert = (override_prefix == 'pitch_servo_insert')
        self._publish_pitch_insert_state()
        response.message = (
            f'ピッチ: {label}(sim={sim_deg}deg, 実機送信={can_deg}deg)' if response.success
            else f'ピッチ: {label}(sim={sim_deg}deg、表示のみ): device_id未設定です')
        return response

    def _on_set_pitch_hold(self, request, response):
        sim_deg = int(self.get_parameter('pitch_servo_hold_deg').value)
        return self._set_pitch(sim_deg, response, '保持姿勢', 'pitch_servo_hold')

    def _on_set_pitch_insert(self, request, response):
        sim_deg = int(self.get_parameter('pitch_servo_insert_deg').value)
        return self._set_pitch(sim_deg, response, '投入姿勢', 'pitch_servo_insert')


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
