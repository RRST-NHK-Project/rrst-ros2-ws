#!/usr/bin/env python3
"""z_joint/r_joint(motor1/motor2差動、M2006+C610)の起動時ホーミングノード。

motor1/motor2はROBOMAS(C610/M2006)内蔵ロータエンコーダのCAN帰還(M{n} angle)を
使う(2026-08-31方針転換、以前は外付けAMTエンコーダ(ENC1/ENC2、CAN_HOST配下
ノード)を使っていたが不使用に変更した)。相対(インクリメンタル)エンコーダの
ため、電源投入毎に基準を喪失する(詳細はnote/hardware_mapping.txt、
note/can_mapping.txt参照)。

z軸・r軸それぞれに上限・下限2個ずつ、合計4個のセンサが実機にある
(motor1/motor2の個別軸ではなく、合成後の軸に対して。2026-08-31確認)。
本ノードが原点較正に使うのは各軸1個(下限側)だけで、上限側は
trajectory_follower_node側で過走防止の安全停止用として別途監視している
(note/can_mapping.txt「z/r上限リミットスイッチ」参照)。
ただし z = mix_k*(m1+m2), r = mix_k*(m1-m2) という合成後の量なので、
ros2canの/zero_channel(motor1またはmotor2を単体でゼロ化するだけの機能)
では正しく較正できない。そこで本ノードは、z軸・r軸それぞれ独立に呼び出せる
2つのサービスを持つ(2026-09-05変更、以前は`start_homing`1個で両方を自動的に
連続実行していたが、組立中や不具合で片方の軸だけをホーミングしたい/もう片方は
まだ動かしたくない、というニーズに対応するため分離した):

  `start_homing_z`:
    1. motor1/motor2を同方向に駆動してz軸の原点センサに当たるまで動かす
       (rが変化しないよう2モータを同じ向きに動かす)
    2. センサ検出時点のロボマス内蔵エンコーダ帰還値から z_raw を計算し、
       z_offset_m = z_ref_value_m - z_raw を求めて反映する

  `start_homing_r`:
    1. motor1/motor2を差動方向に駆動してr軸の原点センサに当たるまで動かす
       (zが変化しないよう2モータを逆向きに動かす)
    2. センサ検出時点のロボマス内蔵エンコーダ帰還値から r_raw を計算し、
       r_offset_m = r_ref_value_m - r_raw を求めて反映する

  どちらも、求めたoffset(z_offset_mまたはr_offset_mの片方のみ)を
  real_joint_bridge_node・trajectory_follower_nodeの両方へSetParameters
  サービスで反映する(前者はsim表示用、後者は実機へのMIT指令生成用。どちらも
  yamlではなくランタイムパラメータとして保持する設計。詳細はreal_joint_bridge_node.py
  冒頭コメント・trajectory_follower_node.pyのrobomas_z_offset_m/r_offset_m参照)。
  もう一方の軸のoffsetには触れないため、z/rは任意の順序・タイミングで独立に
  ホーミングできる(例: r軸がまだ組み立て中でz軸だけ先にホーミングする、等)。
  ただし物理的には差動機構のため、z軸ホーミング中もr軸ホーミング中も
  motor1/motor2は両方とも回転する(役割上「動かしたくない軸」を持つ側の
  モータだけを止めることはできない。note/hardware_mapping.txt参照)。

安全のため、ノード起動時に自動でホーミングは開始しない
(`start_homing_z`/`start_homing_r`(std_srvs/Trigger)サービスを明示的に
呼んだときのみ、それぞれの軸のホーミングを開始する)。
`stop_homing`でいつでも中断しモータを停止できる。

`skip_homing`(std_srvs/Trigger、2026-08-29追加)を呼ぶと、motor1/motor2を
一切駆動せず、現在のエンコーダ値をそのままz_ref_value_m/r_ref_value_m
(原点センサ位置での真値、通常のホーミングと同じパラメータ)の位置とみなして
即座にoffsetを計算・反映する。原点センサの配線・検出値が未確認/未実装
(note/can_mapping.txt「z/r原点センサ」参照)なうちに、機体を手動で(joy等で)
原点センサ位置相当まで動かしてから代わりに使う用途を想定している。
呼び出し前に機体を正しい位置へ物理的に合わせておかないと、以後のz/r値が
すべてズレる点はroot_theta/tip_thetaの原点較正と同じ注意が必要。

trajectory_follower_node(実機出力有効時)は本ノードと同じrobomas_device_idへ
独立にMIT指令をpublishするため、ホーミング中はそのまま放置すると指令が衝突する
(2026-08-29追加)。そこで本ノードはstart_homing_z/start_homing_r時に
trajectory_follower_nodeのpause_robomas_output(std_srvs/Trigger)を呼んで
出力を止め、stop_homing・各軸のホーミング完了・タイムアウト失敗のいずれの
場合もresume_robomas_outputを呼んで再開させる。trajectory_follower_node
未起動(実機出力無効)でもservice呼び出しが失敗するだけで安全(ログ警告のみ、
ホーミング自体は継続)。

ros2can側の前提:
  - ROBOMAS device(motor1/motor2の速度指令・内蔵エンコーダ帰還)の
    `topic_passthrough`をGUIでONにしておかないと、本ノードの/serial_tx_[id]への
    指令や/serial_rx_[id]_unwrappedの帰還が反映されない。
  - CAN_HOST device配下ノードのSW1/SW2をz軸/r軸の原点センサ入力として
    割り当てている想定(note/can_mapping.txt「z/r原点センサ(ホーミング用)」
    参照、未確認なら要修正)。CAN_HOSTはこの原点センサ(リミットスイッチ)の
    ためだけに使う(motor1/motor2の位置取得にはROBOMAS内蔵エンコーダを使うため
    ENC1/ENC2は不使用)。
  - モータ回転方向とz/r方向の対応(*_home_motor*_vel_sign)、原点センサ検出時の
    真値(z_ref_value_m/r_ref_value_m)は実測が必要。実機で少しずつ検証しながら
    調整すること。低速(デフォルト30rpm)・タイムアウト(デフォルト20秒)を
    必ず設定した状態で試すこと。

速度モード(motor1/motor2を直接駆動する本ノードの制御方式)のPIDゲイン
(robomas_vel_kp/ki/kd)・電流上限(robomas_vel_max_current_a)は、2026-09-09までは
firmware(robomas.cpp)のconfig.hppに焼き込まれたコンパイル時固定値だったが、
MITモードのkp/kd/current_ffと同様に毎周期CAN経由でROSから送る可変値にした
(ros2 param set homing_node robomas_vel_kp 0.8 等でホーミング実行中でも変更でき、
_send_velocity/_on_set_parameters参照)。joy操作を将来的にMIT位置モードでは
なく速度モードへ切り替える構想(2026-09-09)で、ファームウェア再書き込み無しに
速度PIDと電流上限を実機で試行錯誤できるようにする狙い。既定値は従来の
config.hpp M2006セクションの値(Kp=0.8, Ki=0.0, Kd=0.0, 電流上限=1.0A)のまま。
"""

import math
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int16MultiArray, Int32MultiArray, String
from std_srvs.srv import Trigger

STATE_IDLE = 'idle'
# pause_robomas_outputの応答待ち中(2026-09-08追加)。モータはまだ駆動しない。
# trajectory_follower_node側が実際に出力を止めたことを確認してからでないと
# motor1/motor2を動かし始めてはいけない(_start_homing_axisのコメント参照)。
STATE_PAUSING_Z = 'pausing_z'
STATE_PAUSING_R = 'pausing_r'
STATE_HOMING_Z = 'homing_z'
STATE_HOMING_R = 'homing_r'
STATE_DONE = 'done'
STATE_FAILED = 'failed'

SLOT_COUNT = 24
# ROBOMASの帰還(angle)スケール。real_joint_bridge_node.py/trajectory_follower_node.pyの
# ROBOMAS_FEEDBACK_POSITION_SCALE_DEGと一致させること(0.1deg/LSB、robomas.cpp参照)。
ROBOMAS_FEEDBACK_POSITION_SCALE_DEG = 0.1

# 速度モード用ゲイン・電流上限スロット(2026-09-09追加。以前はtarget(0-3)しか
# 埋めていなかったが、robomas.cpp側がMIT用に空いていた8-23を速度モードのKp/Ki/Kd・
# 電流上限として読むようになったため、ここも毎周期送る必要がある。送らない
# (全ゼロの)場合、firmware側はKp=Ki=Kd=電流上限=0とみなし出力常に0になる
# (config.hpp「速度モード ゲイン/電流上限のROS可変スロット」参照)。
# LSBスケールはconfig.hppのROBOMAS_VEL_*_LSBと一致させること。
VEL_SLOT_KP = 8
VEL_SLOT_KI = 12
VEL_SLOT_KD = 16
VEL_SLOT_MAX_CURRENT = 20
ROBOMAS_VEL_KP_LSB = 0.001
ROBOMAS_VEL_KI_LSB = 0.001
ROBOMAS_VEL_KD_LSB = 0.0000005
ROBOMAS_VEL_MAX_CURRENT_LSB = 0.001

INT16_MIN, INT16_MAX = -32768, 32767


def clamp_int16(value: float) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(round(value))))


class HomingNode(Node):

    def __init__(self):
        super().__init__('homing_node')

        # ---- CAN_HOST配下ノードのSW1/SW2 (z/r原点センサのみ。motor1/motor2の位置は
        # ROBOMAS内蔵エンコーダを使うためCAN_HOSTのENC1/ENC2は不使用、2026-08-31方針転換) ----
        self.declare_parameter('can_host_device_id', 101)
        self.declare_parameter('can_host_slots_per_node', 5)
        # 要確認(note/can_mapping.txt「z/r原点センサ(ホーミング用)」)。
        # 現状の仮定: z軸原点センサ=SW1、r軸原点センサ=SW2、どちらも同じノード
        # (node_index=1)に載っている。
        self.declare_parameter('z_limit_switch_node_index', 1)
        self.declare_parameter('z_limit_switch_local_index', 0)   # SW1
        self.declare_parameter('r_limit_switch_node_index', 1)
        self.declare_parameter('r_limit_switch_local_index', 1)   # SW2
        self.declare_parameter('switch_triggered_value', 1)       # !digitalRead()相当

        # ---- ROBOMAS device(motor1/motor2の速度指令・内蔵エンコーダ帰還) ----
        self.declare_parameter('robomas_device_id', 21)
        self.declare_parameter('robomas_motor1_index', 0)   # M1 target_velocity/angle
        self.declare_parameter('robomas_motor2_index', 1)   # M2 target_velocity/angle
        self.declare_parameter('pulley_pitch_diameter_mm', 22.92)
        self.declare_parameter('mix_k', 0.5)
        self.declare_parameter('motor1_sign', 1.0)
        self.declare_parameter('motor2_sign', 1.0)

        # ---- 速度モードのゲイン・電流上限 (2026-09-09追加) ----
        # firmware(robomas.cpp)側が毎周期CAN経由で読む可変値。以前はfirmware側の
        # config.hpp(ROBOMAS_KP_VEL等)に焼き込まれたコンパイル時固定値だったが、
        # MITモードのkp/kd/current_ffと同様にROSから調整できるようにした
        # (_send_velocity参照)。既定値はconfig.hpp M2006セクションの従来値
        # (Kp=0.8, Ki=0.0, Kd=0.0, 電流上限=1.0A)に合わせてある。
        self.declare_parameter('robomas_vel_kp', 0.8)
        self.declare_parameter('robomas_vel_ki', 0.0)
        self.declare_parameter('robomas_vel_kd', 0.0)
        self.declare_parameter('robomas_vel_max_current_a', 1.0)

        # ---- ホーミング動作パラメータ(要実機調整) ----
        self.declare_parameter('homing_velocity_rpm', 30.0)
        self.declare_parameter('homing_timeout_sec', 20.0)
        self.declare_parameter('z_home_motor1_vel_sign', -1.0)
        self.declare_parameter('z_home_motor2_vel_sign', -1.0)
        self.declare_parameter('r_home_motor1_vel_sign', 1.0)
        self.declare_parameter('r_home_motor2_vel_sign', -1.0)
        self.declare_parameter('z_ref_value_m', 0.0)   # 要実測: z原点センサ位置での真のz値
        self.declare_parameter('r_ref_value_m', 0.0)   # 要実測: r原点センサ位置での真のr値

        self.declare_parameter('bridge_node_name', 'real_joint_bridge_node')
        self.declare_parameter('trajectory_follower_node_name', 'trajectory_follower_node')
        self.declare_parameter('control_period_sec', 0.05)

        gp = self.get_parameter
        self.can_host_device_id_ = gp('can_host_device_id').value
        self.can_host_slots_per_node_ = gp('can_host_slots_per_node').value

        self.z_sw_node_index_ = gp('z_limit_switch_node_index').value
        self.z_sw_local_index_ = gp('z_limit_switch_local_index').value
        self.r_sw_node_index_ = gp('r_limit_switch_node_index').value
        self.r_sw_local_index_ = gp('r_limit_switch_local_index').value
        self.switch_triggered_value_ = gp('switch_triggered_value').value

        self.robomas_device_id_ = gp('robomas_device_id').value
        self.robomas_motor1_index_ = gp('robomas_motor1_index').value
        self.robomas_motor2_index_ = gp('robomas_motor2_index').value
        self.pulley_radius_m_ = (gp('pulley_pitch_diameter_mm').value / 2.0) / 1000.0
        self.mix_k_ = gp('mix_k').value
        self.motor1_sign_ = gp('motor1_sign').value
        self.motor2_sign_ = gp('motor2_sign').value

        self.vel_kp_ = gp('robomas_vel_kp').value
        self.vel_ki_ = gp('robomas_vel_ki').value
        self.vel_kd_ = gp('robomas_vel_kd').value
        self.vel_max_current_a_ = gp('robomas_vel_max_current_a').value

        self.homing_velocity_rpm_ = gp('homing_velocity_rpm').value
        self.homing_timeout_sec_ = gp('homing_timeout_sec').value
        self.z_home_m1_sign_ = gp('z_home_motor1_vel_sign').value
        self.z_home_m2_sign_ = gp('z_home_motor2_vel_sign').value
        self.r_home_m1_sign_ = gp('r_home_motor1_vel_sign').value
        self.r_home_m2_sign_ = gp('r_home_motor2_vel_sign').value
        self.z_ref_value_m_ = gp('z_ref_value_m').value
        self.r_ref_value_m_ = gp('r_ref_value_m').value

        bridge_node_name = gp('bridge_node_name').value
        trajectory_follower_node_name = gp('trajectory_follower_node_name').value
        control_period = gp('control_period_sec').value

        self.z_sw_slot_ = self.z_sw_node_index_ * self.can_host_slots_per_node_ + self.z_sw_local_index_
        self.r_sw_slot_ = self.r_sw_node_index_ * self.can_host_slots_per_node_ + self.r_sw_local_index_

        self.can_host_data_ = None
        self.robomas_data_ = None
        self.state_ = STATE_IDLE
        self.phase_start_time_ = None
        # stop_homing/連続start_homingで無効化された、届くのが遅れたpause応答
        # コールバックを無視するための世代カウンタ(_start_homing_axis参照)。
        self._homing_request_id_ = 0

        # 状態表示灯(黄色LED、CAN_HOST device_id=101 MULTI1)用。GUIが購読して
        # 「ホーミング中(HOMING_Z/HOMING_R)」「未ホーミング(IDLE)」を判定する
        # (2026-09-08追加、note/note_soki/can_mapping.txt「## 状態表示灯」参照)。
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub_ = self.create_publisher(String, 'homing_state', state_qos)
        self._publish_state()

        self.create_subscription(
            Int32MultiArray, f'serial_rx_{self.can_host_device_id_}_unwrapped',
            self._on_can_host, 10)
        self.create_subscription(
            Int32MultiArray, f'serial_rx_{self.robomas_device_id_}_unwrapped',
            self._on_robomas_feedback, 10)
        self.tx_pub_ = self.create_publisher(
            Int16MultiArray, f'serial_tx_{self.robomas_device_id_}', 10)

        self._set_params_cli = self.create_client(
            SetParameters, f'/{bridge_node_name}/set_parameters')
        self._set_params_cli_traj_ = self.create_client(
            SetParameters, f'/{trajectory_follower_node_name}/set_parameters')
        self._pause_robomas_cli_ = self.create_client(Trigger, 'pause_robomas_output')
        self._resume_robomas_cli_ = self.create_client(Trigger, 'resume_robomas_output')

        self.create_service(Trigger, 'start_homing_z', self._on_start_homing_z)
        self.create_service(Trigger, 'start_homing_r', self._on_start_homing_r)
        self.create_service(Trigger, 'stop_homing', self._on_stop_homing)
        self.create_service(Trigger, 'skip_homing', self._on_skip_homing)

        self.create_timer(control_period, self._on_tick)

        # robomas_vel_kp/ki/kd/max_current_aは起動時にself.vel_*_へ取り込んだ後は
        # 参照されないため、GUI/ros2 param setで変更してもホーミング実行中に
        # 反映されない(trajectory_follower_node.pyの_on_set_parametersと同じ理由)。
        # キャッシュ側も更新することで、ホーミング中でも次回の_send_velocityから
        # 新しい値が反映されるようにする。
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'homing_node started (idle): z_sw=slot{self.z_sw_slot_}, r_sw=slot{self.r_sw_slot_}, '
            f'robomas device_id={self.robomas_device_id_} '
            f'(motor1_index={self.robomas_motor1_index_}, motor2_index={self.robomas_motor2_index_}). '
            f"call 'start_homing_z'/'start_homing_r' service to begin.")

    # ---------------- サービス ----------------

    def _on_start_homing_z(self, request, response):
        return self._start_homing_axis(STATE_PAUSING_Z, STATE_HOMING_Z, 'z', response)

    def _on_start_homing_r(self, request, response):
        return self._start_homing_axis(STATE_PAUSING_R, STATE_HOMING_R, 'r', response)

    def _start_homing_axis(self, pausing_state, homing_state, axis, response):
        """pause_robomas_outputを呼んでから、その応答が届く(=trajectory_follower_node
        が実際にrobomas出力を止めた)のを確認してからモータを動かし始める
        (2026-09-08変更、ユーザー報告: 「Z軸ホーミングでリミットスイッチに当たった
        瞬間に暴走する。時々しか起きない」)。

        以前はpause_robomas_output呼び出しを投げると同時に(応答を待たずに)
        state_をHOMING_Z/HOMING_Rへ変えていたため、次のcontrol_period_sec(既定
        50ms)後の_on_tickで即座にmotor1/motor2への速度指令送信を始めていた。
        pause_robomas_outputの応答(=trajectory_follower_node側でrobomas_paused_
        =Trueになるタイミング)がそれより遅れて届くと、trajectory_follower_node
        がホーミング開始前の位置を保持しようとするMIT位置指令と、homing_node の
        速度指令が同じserial_tx_{robomas_device_id}へ競合して送られる期間が
        発生する。この間、実機はMIT指令(元の位置を維持しようとする力)と速度指令
        (原点センサへ向かう力)の綱引き状態になり、原点センサ到達までの間に
        蓄積したMIT側の位置誤差が、どちらの指令がESP32側の周期(200Hz)に
        より遅く届くか という非決定的なタイミングに応じて、原点センサ到達
        (=homing_node側の指令が0速度へ切り替わる)前後で電流指令に反映され、
        暴走のように見える動きになっていた(pauseが確実に効くかどうかに依存する
        ため再現しないこともある)。"""
        busy_states = (STATE_PAUSING_Z, STATE_PAUSING_R, STATE_HOMING_Z, STATE_HOMING_R)
        if self.state_ in busy_states:
            response.success = False
            response.message = f'already running (state={self.state_})'
            return response
        if self.can_host_data_ is None or self.robomas_data_ is None:
            response.success = False
            response.message = 'serial_rx_*_unwrapped not received yet'
            return response

        self._homing_request_id_ += 1
        request_id = self._homing_request_id_
        self.state_ = pausing_state
        self._publish_state()
        self.get_logger().info(
            f'homing_node: pausing trajectory_follower_node robomas output (phase={axis})')

        def _on_paused():
            # stop_homing、または(理論上は)別のstart_homing呼び出しでこの
            # リクエストが既に無効化されていれば、届いた応答は無視する(状態文字列
            # 自体はpausing_z/pausing_rのように使い回されるため、文字列比較ではなく
            # 世代カウンタで判定する)。
            if request_id != self._homing_request_id_:
                return
            self.state_ = homing_state
            self._publish_state()
            self.phase_start_time_ = time.monotonic()
            self.get_logger().info(f'homing_node: start (phase={axis})')

        self._call_trigger_async(self._pause_robomas_cli_, 'pause_robomas_output', on_done=_on_paused)
        response.success = True
        response.message = f'homing started (phase={axis})'
        return response

    def _on_stop_homing(self, request, response):
        # 保留中のpause応答コールバック(_start_homing_axisの_on_paused)がまだ
        # 届いていなければ無効化する。無効化しないと、この後に別軸のstart_homingを
        # 呼んだ場合、古いpause応答が新しいホーミングのHOMING_Z/HOMING_R遷移を
        # (stateの文字列比較ではなく世代カウンタで防いでいるので)誤って引き起こす
        # ことはないが、念のためここでも世代を進めておく。
        self._homing_request_id_ += 1
        self._send_velocity(0.0, 0.0)
        self.state_ = STATE_IDLE
        self._publish_state()
        self._call_trigger_async(self._resume_robomas_cli_, 'resume_robomas_output')
        self.get_logger().warning('homing_node: stopped by request')
        response.success = True
        response.message = 'stopped'
        return response

    def _on_skip_homing(self, request, response):
        """motor1/motor2を駆動せず、現在位置をz_ref_value_m/r_ref_value_mの位置と
        みなしてoffsetを即座に確定する(ファイル冒頭docstring参照)。呼び出し前に
        機体を原点センサ位置相当へ物理的に合わせておくこと。"""
        if self.state_ in (STATE_PAUSING_Z, STATE_PAUSING_R, STATE_HOMING_Z, STATE_HOMING_R):
            response.success = False
            response.message = f'ホーミング動作中は使えません(state={self.state_})'
            return response
        if self.can_host_data_ is None or self.robomas_data_ is None:
            response.success = False
            response.message = 'serial_rx_*_unwrapped not received yet'
            return response

        m1, m2 = self._current_motor_joints()
        z_raw = self.mix_k_ * (m1 + m2)
        r_raw = self.mix_k_ * (m1 - m2)
        z_offset_m = self.z_ref_value_m_ - z_raw
        r_offset_m = self.r_ref_value_m_ - r_raw
        self._apply_offset('z', z_offset_m)
        self._apply_offset('r', r_offset_m)
        self.state_ = STATE_DONE
        self._publish_state()
        self.get_logger().warning(
            f'homing_node: skip_homing実行(モータ駆動なし)。現在位置を'
            f'z={self.z_ref_value_m_:.5f}, r={self.r_ref_value_m_:.5f} とみなしました '
            f'(z_offset_m={z_offset_m:.5f}, r_offset_m={r_offset_m:.5f})')
        response.success = True
        response.message = (
            f'skip_homing: z_offset_m={z_offset_m:.5f}, r_offset_m={r_offset_m:.5f} '
            f'を反映しました(機体が正しい位置にあった前提)')
        return response

    def _publish_state(self):
        msg = String()
        msg.data = self.state_
        self.state_pub_.publish(msg)

    def _call_trigger_async(self, client, label, on_done=None):
        """on_done: 成功/失敗を問わず、応答が確定した後(またはサービス未提供で
        即座に諦めた場合はその場)に呼ばれるコールバック(2026-09-08追加、
        _start_homing_axisがpause_robomas_outputの完了を待ってからモータを
        動かし始めるのに使う)。trajectory_follower_node未起動(実機出力無効)でも
        安全に無視できるよう、サービス未提供時はログ警告のみでホーミング自体は
        継続する(on_doneはこの場合も呼ぶ)。"""
        if not client.service_is_ready():
            self.get_logger().warning(f'homing_node: {label} service not available, skipped')
            if on_done is not None:
                on_done()
            return

        def _done(fut):
            try:
                res = fut.result()
                if not res.success:
                    self.get_logger().warning(f'homing_node: {label} failed: {res.message}')
            except Exception as exc:
                self.get_logger().error(f'homing_node: {label} call error: {exc}')
            finally:
                if on_done is not None:
                    on_done()

        client.call_async(Trigger.Request()).add_done_callback(_done)

    # ---------------- パラメータ ----------------

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'robomas_vel_kp':
                self.vel_kp_ = float(p.value)
            elif p.name == 'robomas_vel_ki':
                self.vel_ki_ = float(p.value)
            elif p.name == 'robomas_vel_kd':
                self.vel_kd_ = float(p.value)
            elif p.name == 'robomas_vel_max_current_a':
                self.vel_max_current_a_ = float(p.value)
        return SetParametersResult(successful=True)

    # ---------------- センサ購読 ----------------

    def _on_can_host(self, msg: Int32MultiArray):
        self.can_host_data_ = msg.data
        # ホーミング中は、次の_on_tick(最大control_period_sec後、既定50ms)を
        # 待たずにCAN_HOSTの帰還が届いた瞬間に原点センサをチェックする
        # (2026-09-08追加、ユーザー報告: 「R軸の原点取りがセンサが反応していない
        # ような挙動(そのままつっこむ)をする。しっかりと止まることもある」)。
        # IO_SW_Input()(ros2can firmware pin_ctrl_task.cpp)はSWの生値を
        # デバウンス無しでそのまま送っており、ポーリング(_on_tick任せ)だと
        # センサが実際に検出されてから停止指令を送るまでに最大control_period_sec
        # ぶんの遅延が上乗せされる。原点センサの検出範囲(機構が停止指令に反応
        # するまでに進める余裕)が狭いと、この遅延の間に通り過ぎてしまい
        # 「センサ無反応で突っ込む」ように見える。帰還到着のたびに即座にチェック
        # することでこの遅延を無くす(タイミング次第で再現したりしなかったりする
        # のは、この遅延が固定値ではなくCAN/シリアル中継やexecutorのスケジュー
        # リングに依存する非決定的な量だったため)。
        self._check_homing_switch()

    def _on_robomas_feedback(self, msg: Int32MultiArray):
        self.robomas_data_ = msg.data

    def _current_motor_joints(self):
        m1_deg = self.robomas_data_[self.robomas_motor1_index_] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        m2_deg = self.robomas_data_[self.robomas_motor2_index_] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        motor1_joint = self.motor1_sign_ * math.radians(m1_deg) * self.pulley_radius_m_
        motor2_joint = self.motor2_sign_ * math.radians(m2_deg) * self.pulley_radius_m_
        return motor1_joint, motor2_joint

    def _switch_triggered(self, slot):
        return self.can_host_data_[slot] == self.switch_triggered_value_

    def _check_homing_switch(self):
        """state_がHOMING_Z/HOMING_Rのとき、対応する原点センサが検出されて
        いれば停止・offset確定まで行う。_on_tick(タイムアウト監視・速度指令の
        継続送信)と_on_can_host(CAN_HOST帰還到着時の即時チェック、2026-09-08
        追加)の両方から呼ばれる(_on_tick側で既にDONEへ遷移済みなら、この
        メソッド冒頭のstate_チェックで何もせず戻るので二重発火はしない)。
        センサ検出でoffsetを確定・停止した場合True、それ以外はFalseを返す。"""
        if self.state_ == STATE_HOMING_Z:
            axis, slot, ref_value = 'z', self.z_sw_slot_, self.z_ref_value_m_
        elif self.state_ == STATE_HOMING_R:
            axis, slot, ref_value = 'r', self.r_sw_slot_, self.r_ref_value_m_
        else:
            return False
        if self.can_host_data_ is None or self.robomas_data_ is None:
            return False
        if not self._switch_triggered(slot):
            return False

        self._send_velocity(0.0, 0.0)
        m1, m2 = self._current_motor_joints()
        raw = self.mix_k_ * (m1 + m2) if axis == 'z' else self.mix_k_ * (m1 - m2)
        offset_m = ref_value - raw
        self.get_logger().info(
            f'homing_node: {axis}-axis limit reached ({axis}_raw={raw:.5f}, '
            f'{axis}_offset_m={offset_m:.5f}). applying offset')
        self.state_ = STATE_DONE
        self._publish_state()
        self._apply_offset(
            axis, offset_m,
            on_all_done=lambda: self._call_trigger_async(
                self._resume_robomas_cli_, 'resume_robomas_output'))
        return True

    # ---------------- 制御ループ ----------------

    def _send_velocity(self, motor1_rpm, motor2_rpm):
        data = [0] * SLOT_COUNT
        data[self.robomas_motor1_index_] = int(motor1_rpm)
        data[self.robomas_motor2_index_] = int(motor2_rpm)
        # 速度モードのKp/Ki/Kd・電流上限も毎回送る(送らない=全ゼロのままだと
        # firmware側は出力常に0とみなし、target(上記)を送っても動かない。
        # config.hpp「速度モード ゲイン/電流上限のROS可変スロット」参照)。
        for motor_index in (self.robomas_motor1_index_, self.robomas_motor2_index_):
            data[VEL_SLOT_KP + motor_index] = clamp_int16(self.vel_kp_ / ROBOMAS_VEL_KP_LSB)
            data[VEL_SLOT_KI + motor_index] = clamp_int16(self.vel_ki_ / ROBOMAS_VEL_KI_LSB)
            data[VEL_SLOT_KD + motor_index] = clamp_int16(self.vel_kd_ / ROBOMAS_VEL_KD_LSB)
            data[VEL_SLOT_MAX_CURRENT + motor_index] = clamp_int16(
                self.vel_max_current_a_ / ROBOMAS_VEL_MAX_CURRENT_LSB)
        msg = Int16MultiArray()
        msg.data = data
        self.tx_pub_.publish(msg)

    def _on_tick(self):
        if self.state_ not in (STATE_HOMING_Z, STATE_HOMING_R):
            return
        if self.can_host_data_ is None or self.robomas_data_ is None:
            return

        elapsed = time.monotonic() - self.phase_start_time_
        if elapsed > self.homing_timeout_sec_:
            failed_phase = self.state_
            self._send_velocity(0.0, 0.0)
            self.state_ = STATE_FAILED
            self._publish_state()
            self._call_trigger_async(self._resume_robomas_cli_, 'resume_robomas_output')
            self.get_logger().error(
                f'homing_node: TIMEOUT during {failed_phase} (>{self.homing_timeout_sec_}s). '
                f'motors stopped.')
            return

        # センサ検出チェックは_on_can_host側で帰還到着のたびに既に行っているが、
        # (a)_on_tickの方が先に実行された場合の取りこぼし防止、(b)何らかの理由で
        # _on_can_hostが呼ばれなかった場合の保険として、ここでも呼ぶ
        # (_check_homing_switch冒頭のstate_チェックにより二重発火はしない)。
        if self._check_homing_switch():
            return

        if self.state_ == STATE_HOMING_Z:
            self._send_velocity(
                self.z_home_m1_sign_ * self.homing_velocity_rpm_,
                self.z_home_m2_sign_ * self.homing_velocity_rpm_)
        elif self.state_ == STATE_HOMING_R:
            self._send_velocity(
                self.r_home_m1_sign_ * self.homing_velocity_rpm_,
                self.r_home_m2_sign_ * self.homing_velocity_rpm_)

    def _apply_offset(self, axis, offset_m, on_all_done=None):
        """axis('z'/'r')のoffsetだけをreal_joint_bridge_node・trajectory_follower_node
        双方へ反映する(2026-09-05変更、以前は両軸まとめて反映していたが、z/rを
        独立にホーミングできるようにしたため片方ずつ送るようにした。もう一方の
        軸のoffsetパラメータには触れない)。real_joint_bridge_node(sim表示用)と
        trajectory_follower_node(実機MIT指令用)の両方が独自にoffsetを保持している
        (2026-08-29追加。trajectory_follower_node側はrobomas_z/r_offset_mという
        パラメータ名で、実機出力無効(robomas_device_id未設定)なら値自体は
        使われないだけなので、未起動でも安全に無視できる)。

        on_all_done: 両方のSetParameters呼び出しが完了(成功/失敗問わず)した後に
        呼ばれるコールバック(2026-09-08追加)。呼び出し元はこれを使って
        resume_robomas_outputをoffset反映後まで遅らせること。trajectory_follower_node
        は`robomas_paused_`がFalseに戻った瞬間に実機帰還でのpos_追従を止めるため
        (_on_robomas_feedback参照)、new offsetの反映より先にresumeが届くと
        pos_が古いoffset基準のまま凍結され、MIT指令がoffset分だけ瞬時にステップ
        してしまう(実機急動作の原因になっていたレースコンディション)。"""
        bridge_param = 'z_offset_m' if axis == 'z' else 'r_offset_m'
        traj_param = 'robomas_z_offset_m' if axis == 'z' else 'robomas_r_offset_m'

        remaining = [2]

        def _one_done():
            remaining[0] -= 1
            if remaining[0] == 0 and on_all_done is not None:
                on_all_done()

        self._apply_offset_to(self._set_params_cli, bridge_param, offset_m,
                               'real_joint_bridge_node', _one_done)
        self._apply_offset_to(self._set_params_cli_traj_, traj_param, offset_m,
                               'trajectory_follower_node', _one_done)

    def _apply_offset_to(self, client, param_name, offset_m, label, on_done):
        if not client.service_is_ready():
            self.get_logger().error(
                f'homing_node: {label} set_parameters service not available, '
                f'offset NOT applied. Is {label} running?')
            on_done()
            return
        params = [
            Parameter(name=param_name,
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                            double_value=float(offset_m))),
        ]
        future = client.call_async(SetParameters.Request(parameters=params))

        def _done(fut):
            try:
                fut.result()
                self.get_logger().info(f'homing_node: {param_name} applied to {label}')
            except Exception as exc:
                self.get_logger().error(f'homing_node: failed to apply {param_name} to {label}: {exc}')
            finally:
                on_done()

        future.add_done_callback(_done)


def main(args=None):
    rclpy.init(args=args)
    node = HomingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
