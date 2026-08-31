#!/usr/bin/env python3
"""
soki_sim: 関節の目標値(/joint_targets)を台形速度プロファイルで滑らかに追従させ、
現在値を/mixed_joint_states(joint_state_publisher(_gui)のsource_list)へ
publishし続けるノード。

これまでcommand_gui_nodeが目標値を直接/mixed_joint_statesへpublishしていたため
soki_sim上の関節は瞬時にジャンプしていたが、本ノードを間に挟むことで
max_velocity/max_accelerationで制限された現実的な速度で動くようになる
(command_gui_node側は/joint_targetsへpublishするよう変更済み)。

1回のtarget_callbackで同時に届いた関節群は同時到達するよう、各関節の
(自分のmax_velocity/max_accelerationでの)所要時間を計算し、最も時間がかかる
関節に他の関節を合わせて速度・加速度を時間軸方向にスケールする(台形プロファイル
の形は保ったまま引き伸ばす)。command_gui_nodeはtheta/z/rを毎回1つのJointState
にまとめて送るため、これらは常に同時到達になる。

同じ軌道(位置・速度)を、実機のCubeMars AKシリーズ(MITモード)へもそのまま
指令できる。cubemars_joint_names等のパラメータで対象関節・device_id・
モータ番号(0-3, ros2canのM1-M4に対応)・reduction(関節<->アクチュエータ軸の
減速比、note/hardware_mapping.txt参照)を指定すると、ros2canのトピック規約
(serial_tx_[ID] へInt16MultiArray(24スロット)をpublishすると外部指令として
反映される。ros2can/ros2can/ros_backend.py参照)に従ってMIT指令フレームを
毎周期publishする。device_id/motor_indexの対応は実機配線に依存するため、
未設定(デフォルト)の関節は実機への出力を行わずsoki_simの表示のみ更新する。

MITモードの各スロットのスケールはros2can/firmware/xiao-esp32-s3_can2io/src/
cubemars.cppのコメントと一致させること
  (target: 0.1deg/LSB, mit_velocity: 0.01rad/s/LSB, mit_kp: 0.1/LSB,
   mit_kd: 0.01/LSB, mit_torque_ff: 0.01N・m/LSB)。
target/mit_velocityはアクチュエータ軸(CubeMars本体側)の値であり、本ノード内部の
self.pos_/self.vel_(関節角度)に cubemars_reduction を掛けて変換してから送信する
(real_joint_bridge_nodeの joint = actuator / reduction の逆変換、2026-08-27修正)。
また、最初の/joint_targets受信より前は serial_rx_[device_id]_unwrapped の実機
帰還値でself.pos_を追従させ、起動直後の内部状態(0.0)と実機の実際の角度との
ズレによる意図しない位置ジャンプを防ぐ(_on_cubemars_feedback参照)。

root_theta_jointについては、/set_root_theta_origin(std_srvs/Trigger)サービスで
CubeMars本体(AK40-10)へSet Origin(永久原点、フラッシュ保存)CANコマンドを送信できる
(2026-08-27追加、control_mode=3、ros2can/firmware側の対応実装はcubemars.cpp参照)。
呼び出すと数周期(ORIGIN_HOLD_CYCLES)だけ通常のMIT指令を止めてSET_ORIGINモードを
送る。これにより実機エンコーダ自体の原点が電源off/onを跨いで保持されるため、
real_joint_bridge_node側のroot_theta_offset_radによるソフトウェア補正は廃止した
(note/hardware_mapping.txt参照)。

/joint_targetsの送信元は自動(command_gui_node)とjoy_teleop_node(手動操作)の
2系統があり、control_modeパラメータ('auto'/'manual'/'both')でどちらを受け付ける
かを切り替えられる(command_gui_nodeの「動作モード」パネルから変更可能)。
送信元の判別にはJointStateメッセージのheader.frame_id('auto'または'manual'、
未設定は'auto'扱い)を使う。台形速度プロファイルによる滑らかな追従はどちらの
送信元でも同じロジックが適用される。

z_joint/r_jointについても、実機のロボマス(M2006+C610、motor1/motor2の差動機構、
note/hardware_mapping.txt参照)へMIT(位置PD制御)モードで同時に指令できる
(2026-08-29追加、ros2canのMODE_ROBOMASにMITモードが実装されたことを受けて
root_theta/tip_theta(CubeMars MIT)と対称的に追加)。robomas_device_id等の
パラメータで有効化する(未設定=device_id0のデフォルトのままなら実機出力無効)。
z/r(joint角度)はそれぞれ独立に台形プロファイルされた後、motor_mixer_nodeと
同じ式(m1=(z+r)/(2*mix_k), m2=(z-r)/(2*mix_k))でmotor1/motor2側(アクチュエータ軸)
へ変換してから送信する。位置フィードバックはCubeMars側と異なりロボマス内蔵の
ロータエンコーダを基準に割り切る(z/rの真値である外付けAMTエンコーダとの
バックラッシュ差分は無視する設計判断、note/hardware_mapping.txt参照)。

homing_node(z/rの起動時ホーミング)は本ノードと同じdevice(robomas_device_id)へ
独立に速度指令をpublishするため、ホーミング中に本ノードのMIT指令と衝突する。
これを避けるため、pause_robomas_output/resume_robomas_output(std_srvs/Trigger)
サービスを設け、ホーミング中は本ノード側のrobomas出力を丸ごと止められるように
した(root_theta原点設定の_origin_pending_と同様、外部ノードから能動的に
出力を止めるパターン)。homing_node側がstart_homing時にpause、
stop_homing/完了/タイムアウト失敗時にresumeを呼ぶ。

z/r軸にはそれぞれ上限・下限のリミットスイッチがある(homing_nodeが原点較正に
使う下限側とは別に、上限側は過走防止の安全停止専用、2026-08-31追加)。
本ノードは*_limit_switch_device_id/*_node_index/*_local_indexパラメータ
(z/r × lower/upper の4系統、device_id=0で該当スイッチ無効)でCAN_HOSTの
帰還を直接監視し、timer_callbackでトリガーされている方向への移動だけを
ロックする(反対方向への後退は許可、_limit_triggered参照)。
"""
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

CUBEMARS_MODE_MIT = 2
CUBEMARS_POSITION_SCALE_DEG = 0.1  # 帰還/MIT指令とも0.1deg/LSB(cubemars.cpp参照)
CUBEMARS_MODE_SET_ORIGIN = 3  # ros2can/firmware/.../cubemars.cppのcontrol_mode enumと一致させること
CUBEMARS_ORIGIN_MODE_PERMANENT = 1  # target流用: 0=一時原点/1=永久原点(フラッシュ保存)/2=デフォルト復元
CUBEMARS_SLOT_COUNT = 24

# ROBOMAS(z_joint/r_joint、motor1/motor2)のMITモード。cubemarsとスケールが異なる点に
# 注意(robomas.cpp/config.hppのROBOMAS_MIT_*参照)。
ROBOMAS_MODE_VELOCITY = 0
ROBOMAS_MODE_MIT = 1
ROBOMAS_MIT_POSITION_CMD_SCALE_DEG = 1.0     # 指令(target): 1deg/LSB(帰還よりだいぶ粗い、範囲確保のため)
ROBOMAS_FEEDBACK_POSITION_SCALE_DEG = 0.1    # 帰還(angle): 0.1deg/LSB(cubemarsの帰還と同じ)
ROBOMAS_SLOT_COUNT = 24

# /set_root_theta_originサービス呼び出し後、Set Originコマンドの送信を保証するために
# 通常のMIT指令を止めてSET_ORIGINモードを保持する周期数(update_rate_hzでの周期数)。
# ros2can側firmwareは実際のCANコマンド送信をエッジ検出で1回だけ行うため、この間に
# 少なくとも1回はcubemarsTask(200Hz)の処理サイクルへ届けば十分。
ORIGIN_HOLD_CYCLES = 10

INT16_MIN, INT16_MAX = -32768, 32767

VALID_CONTROL_MODES = ('auto', 'manual', 'both')


def clamp_int16(value: float) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(round(value))))


def trap_step(pos: float, vel: float, target: float, max_vel: float, max_accel: float, dt: float):
    """加速度max_accel・速度max_velで制限した台形速度プロファイルで1ステップ進める。

    残り距離ちょうどでvel=0にできる速度(sqrt(2*max_accel*|error|)、等加速度運動の
    公式 v^2=2ad から導出)を速度上限として使うことで、目標到達時にオーバーシュート
    しないよう自動的に減速する。
    """
    error = target - pos
    if max_accel <= 0.0 or max_vel <= 0.0:
        return target, 0.0

    brake_vel = math.sqrt(max(0.0, 2.0 * max_accel * abs(error)))
    vel_limit = min(max_vel, brake_vel)
    desired_vel = math.copysign(vel_limit, error) if error != 0.0 else 0.0

    max_dv = max_accel * dt
    dv = max(-max_dv, min(max_dv, desired_vel - vel))
    new_vel = vel + dv
    new_pos = pos + new_vel * dt

    if (error >= 0.0 and new_pos >= target) or (error < 0.0 and new_pos <= target):
        return target, 0.0
    return new_pos, new_vel


def move_time(distance: float, max_vel: float, max_accel: float) -> float:
    """distance移動するのにtrap_stepの台形プロファイルで要する時間(理論値)。

    加速区間で最高速度max_velに到達できる距離(max_vel^2/max_accel)以上なら
    台形(加速+等速+減速)、届かなければ三角形(加減速のみ)の所要時間になる。
    """
    d = abs(distance)
    if d < 1e-9 or max_vel <= 0.0 or max_accel <= 0.0:
        return 0.0
    accel_dist = max_vel * max_vel / max_accel
    if d >= accel_dist:
        return 2.0 * max_vel / max_accel + (d - accel_dist) / max_vel
    return 2.0 * math.sqrt(d / max_accel)


class TrajectoryFollowerNode(Node):

    def __init__(self):
        super().__init__('trajectory_follower_node')

        self.declare_parameter('joint_names', ['root_theta_joint', 'z_joint', 'r_joint'])
        self.declare_parameter('max_velocity', [1.0, 0.05, 0.05])
        self.declare_parameter('max_acceleration', [2.0, 0.1, 0.1])
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('input_topic', 'joint_targets')
        self.declare_parameter('output_topic', 'mixed_joint_states')
        # 'auto'=command_gui_nodeのみ受付, 'manual'=joy_teleop_nodeのみ受付, 'both'=両方受付
        self.declare_parameter('control_mode', 'auto')

        # 実機CubeMars(MITモード)への同時出力。対象関節ごとに device_id/motor_index/
        # kp/kd/torque_ffを同じ長さの配列で指定する(未設定=空配列ならsoki_sim表示のみ)。
        # device_id・motor_index(0-3=M1-M4)は実機配線に依存するため、確認できてから
        # launch引数またはros2 paramで設定すること。
        # 空配列(デフォルト=実機出力無効)ではrclpyが要素型を推定できないため、
        # dynamic_typing=Trueでオーバーライド時の型(string/integer/double array)を許容する。
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('cubemars_joint_names', [], dyn)
        self.declare_parameter('cubemars_device_ids', [], dyn)
        self.declare_parameter('cubemars_motor_indices', [], dyn)
        self.declare_parameter('cubemars_kp', [], dyn)
        self.declare_parameter('cubemars_kd', [], dyn)
        self.declare_parameter('cubemars_torque_ff', [], dyn)
        # 関節角度[joint]とCubeMars本体(アクチュエータ軸)角度[actuator]の変換比:
        # actuator_deg = joint_deg * reduction (real_joint_bridge_nodeの
        # joint = actuator / reduction の逆)。note/hardware_mapping.txt参照。
        self.declare_parameter('cubemars_reduction', [], dyn)

        # 実機ROBOMAS(MITモード)への同時出力。z_joint/r_jointをmotor1/motor2へ
        # 変換して送信する(cubemars_*と異なり対象がz/r固定のためスカラーパラメータ、
        # モジュール先頭のROBOMAS_MIT_*スケール定数参照)。device_id=0(既定)は
        # 実機出力無効を意味する(0は実在のCAN device_idとして使わない前提)。
        self.declare_parameter('robomas_device_id', 0)
        self.declare_parameter('robomas_motor1_index', 0)   # M1
        self.declare_parameter('robomas_motor2_index', 1)   # M2
        self.declare_parameter('robomas_kp', 0.0)
        self.declare_parameter('robomas_kd', 0.0)
        self.declare_parameter('robomas_current_ff', 0.0)
        self.declare_parameter('robomas_z_joint', 'z_joint')
        self.declare_parameter('robomas_r_joint', 'r_joint')
        self.declare_parameter('robomas_mix_k', 0.5)
        self.declare_parameter('robomas_pulley_pitch_diameter_mm', 22.92)
        self.declare_parameter('robomas_motor1_sign', 1.0)
        self.declare_parameter('robomas_motor2_sign', 1.0)
        # homing_nodeがSetParametersで実行時に更新するランタイム専用オフセット
        # (real_joint_bridge_nodeのz_offset_m/r_offset_mと同じ役割・符号規約を、
        # 本ノード側のMIT指令生成用に別途持つ。real_joint_bridge_node.pyの
        # z = mix_k*(m1+m2) + z_offset_m と揃えること)。
        self.declare_parameter('robomas_z_offset_m', 0.0)
        self.declare_parameter('robomas_r_offset_m', 0.0)

        # z/r軸それぞれの上限・下限リミットスイッチ(CAN_HOST経由、過走防止の
        # 安全停止用、2026-08-31追加)。homing_nodeが原点較正に使う原点センサ
        # (通常は下限側)とは別に、本ノードは4個(z上限/z下限/r上限/r下限)を
        # 独立に監視し、トリガーされた方向への移動だけをロックする(反対方向への
        # 後退は許可、_limit_triggered/timer_callback参照)。device_id=0(既定)は
        # そのスイッチ未配線・無効を意味する。node_index/local_indexの意味は
        # homing_node.py/note/can_mapping.txtの原点センサと同じ
        # (local_index: 0=SW1/1=SW2/2=SW3)。
        self.declare_parameter('limit_switch_can_host_slots_per_node', 5)
        self.declare_parameter('limit_switch_triggered_value', 1)
        for _axis, _direction in (('z', 'lower'), ('z', 'upper'), ('r', 'lower'), ('r', 'upper')):
            _prefix = f'{_axis}_{_direction}_limit_switch'
            self.declare_parameter(f'{_prefix}_device_id', 0)
            self.declare_parameter(f'{_prefix}_node_index', 0)
            self.declare_parameter(f'{_prefix}_local_index', 0)

        self.joint_names_ = list(self.get_parameter('joint_names').value)
        max_vel = list(self.get_parameter('max_velocity').value)
        max_accel = list(self.get_parameter('max_acceleration').value)
        if len(max_vel) != len(self.joint_names_) or len(max_accel) != len(self.joint_names_):
            raise ValueError('max_velocity/max_acceleration must have the same length as joint_names')

        self.control_mode_ = self.get_parameter('control_mode').value
        if self.control_mode_ not in VALID_CONTROL_MODES:
            raise ValueError(f'control_mode must be one of {VALID_CONTROL_MODES}')

        self.max_vel_ = dict(zip(self.joint_names_, max_vel))
        self.max_accel_ = dict(zip(self.joint_names_, max_accel))
        # 同時到達スケーリング後の実効値。target_callbackで毎回更新される
        # (未受信時はmax_vel_/max_accel_と同じ=通常の単軸プロファイル)。
        self.eff_max_vel_ = dict(self.max_vel_)
        self.eff_max_accel_ = dict(self.max_accel_)
        self.pos_ = {name: 0.0 for name in self.joint_names_}
        self.vel_ = {name: 0.0 for name in self.joint_names_}
        self.target_ = {name: 0.0 for name in self.joint_names_}
        self.has_target_ = False

        self._setup_cubemars_outputs()
        self._setup_robomas_outputs()
        self._setup_limit_switches()
        self.create_service(Trigger, 'set_root_theta_origin', self._on_set_root_theta_origin)

        # max_velocity/max_accelerationは起動時にself.max_vel_/max_accel_へ
        # 取り込んだ後は参照されないため、command_gui_node等がros2 param set(GUIの
        # 「軌道生成パラメータ」パネル含む)で変更しても反映されなかった。
        # on_set_parameters_callbackでキャッシュ側も更新することで実行中に反映させる。
        self.add_on_set_parameters_callback(self._on_set_parameters)

        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.dt_ = 1.0 / update_rate_hz
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub_ = self.create_publisher(JointState, output_topic, 10)
        self.sub_ = self.create_subscription(
            JointState, input_topic, self.target_callback, 10)
        self.timer_ = self.create_timer(self.dt_, self.timer_callback)

        self.get_logger().info(
            f'trajectory_follower_node started: {input_topic} -> {output_topic} '
            f'(joints={self.joint_names_}, rate={update_rate_hz}Hz, '
            f'control_mode={self.control_mode_}, cubemars_joints={list(self.cubemars_.keys())}, '
            f'robomas_enabled={self.robomas_ is not None}, '
            f'limit_switches={sorted("_".join(k) for k in self._limit_switches_.keys())})')

    def _setup_cubemars_outputs(self):
        names = list(self.get_parameter('cubemars_joint_names').value)
        device_ids = list(self.get_parameter('cubemars_device_ids').value)
        motor_indices = list(self.get_parameter('cubemars_motor_indices').value)
        kp = list(self.get_parameter('cubemars_kp').value)
        kd = list(self.get_parameter('cubemars_kd').value)
        torque_ff = list(self.get_parameter('cubemars_torque_ff').value)
        reduction = list(self.get_parameter('cubemars_reduction').value)

        lengths = {len(names), len(device_ids), len(motor_indices), len(kp), len(kd), len(torque_ff),
                   len(reduction)}
        if len(names) > 0 and lengths != {len(names)}:
            raise ValueError(
                'cubemars_joint_names/device_ids/motor_indices/kp/kd/torque_ff/reduction '
                'must all have the same length')
        for name in names:
            if name not in self.joint_names_:
                raise ValueError(f'cubemars_joint_names entry "{name}" is not in joint_names')

        # device_idごとに24スロットのバッファを保持し、対象モータのスロットのみ毎周期更新する
        # (同じdeviceに他モータが同居しない前提。ros2can/note/claude.txtのMODE_CUBEMARS参照)。
        self.cubemars_ = {}
        self.device_buffers_ = {}
        self.device_publishers_ = {}
        self.device_feedback_subs_ = {}
        for name, device_id, motor_index, kp_v, kd_v, tff_v, red_v in zip(
                names, device_ids, motor_indices, kp, kd, torque_ff, reduction):
            self.cubemars_[name] = {
                'device_id': int(device_id),
                'motor_index': int(motor_index),
                'kp': float(kp_v),
                'kd': float(kd_v),
                'torque_ff': float(tff_v),
                'reduction': float(red_v),
            }
            if device_id not in self.device_buffers_:
                self.device_buffers_[device_id] = [0] * CUBEMARS_SLOT_COUNT
                self.device_publishers_[device_id] = self.create_publisher(
                    Int16MultiArray, f'serial_tx_{device_id}', 10)
                self.device_feedback_subs_[device_id] = self.create_subscription(
                    Int32MultiArray, f'serial_rx_{device_id}_unwrapped',
                    lambda msg, did=device_id: self._on_cubemars_feedback(msg, did), 10)

        # /set_root_theta_originで起動した「原点設定モード保持」の残り周期数
        # (関節名 -> 残りcycle数)。0または未登録なら通常のMIT指令を送る。
        self._origin_pending_ = {}

    def _on_cubemars_feedback(self, msg: Int32MultiArray, device_id: int):
        # 最初の/joint_targets受信(has_target_=True)より前は、実機からの帰還値で
        # self.pos_を追従させておく。これをしないとノード起動直後の内部状態(0.0)と
        # 実機の実際の角度がズレたまま最初の目標へ台形プロファイルが走り、
        # 実機側に意図しない大きな位置ジャンプ(Kp*(p_des-p)のステップ入力)が
        # 発生する。目標を一度でも受け取ったら以後はプロファイル側の値を信用し、
        # 帰還での上書きはしない。
        if self.has_target_:
            return
        for name, cfg in self.cubemars_.items():
            if cfg['device_id'] != device_id:
                continue
            raw_deg = msg.data[cfg['motor_index']] * CUBEMARS_POSITION_SCALE_DEG
            joint_rad = math.radians(raw_deg) / cfg['reduction']
            self.pos_[name] = joint_rad
            self.vel_[name] = 0.0
            self.target_[name] = joint_rad

    def _setup_robomas_outputs(self):
        device_id = int(self.get_parameter('robomas_device_id').value)
        self.robomas_ = None
        self.robomas_paused_ = False
        if device_id == 0:
            return

        z_name = self.get_parameter('robomas_z_joint').value
        r_name = self.get_parameter('robomas_r_joint').value
        if z_name not in self.joint_names_ or r_name not in self.joint_names_:
            raise ValueError('robomas_z_joint/robomas_r_joint must be in joint_names')

        self.robomas_ = {
            'device_id': device_id,
            'motor1_index': int(self.get_parameter('robomas_motor1_index').value),
            'motor2_index': int(self.get_parameter('robomas_motor2_index').value),
            'kp': float(self.get_parameter('robomas_kp').value),
            'kd': float(self.get_parameter('robomas_kd').value),
            'current_ff': float(self.get_parameter('robomas_current_ff').value),
            'z_joint': z_name,
            'r_joint': r_name,
            'mix_k': float(self.get_parameter('robomas_mix_k').value),
            'pulley_radius_m': (float(self.get_parameter('robomas_pulley_pitch_diameter_mm').value)
                                 / 2.0 / 1000.0),
            'motor1_sign': float(self.get_parameter('robomas_motor1_sign').value),
            'motor2_sign': float(self.get_parameter('robomas_motor2_sign').value),
        }
        self.robomas_pub_ = self.create_publisher(
            Int16MultiArray, f'serial_tx_{device_id}', 10)
        self.create_subscription(
            Int32MultiArray, f'serial_rx_{device_id}_unwrapped', self._on_robomas_feedback, 10)
        self.create_service(Trigger, 'pause_robomas_output', self._on_pause_robomas_output)
        self.create_service(Trigger, 'resume_robomas_output', self._on_resume_robomas_output)

    def _setup_limit_switches(self):
        """z/r軸それぞれの上限・下限リミットスイッチ(CAN_HOST経由)を監視する。
        robomas_device_id未設定(実機出力無効)なら対象のz_joint/r_joint自体が
        無いため何もしない。"""
        self._limit_switches_ = {}
        self._limit_switch_can_host_data_ = {}
        self._limit_switch_subs_ = {}
        self._limit_switch_prev_triggered_ = {}
        if self.robomas_ is None:
            return

        slots_per_node = int(self.get_parameter('limit_switch_can_host_slots_per_node').value)
        self.limit_switch_triggered_value_ = int(
            self.get_parameter('limit_switch_triggered_value').value)
        joint_for_axis = {'z': self.robomas_['z_joint'], 'r': self.robomas_['r_joint']}

        for axis, direction in (('z', 'lower'), ('z', 'upper'), ('r', 'lower'), ('r', 'upper')):
            prefix = f'{axis}_{direction}_limit_switch'
            device_id = int(self.get_parameter(f'{prefix}_device_id').value)
            if device_id == 0:
                continue
            node_index = int(self.get_parameter(f'{prefix}_node_index').value)
            local_index = int(self.get_parameter(f'{prefix}_local_index').value)
            key = (axis, direction)
            self._limit_switches_[key] = {
                'device_id': device_id,
                'slot': node_index * slots_per_node + local_index,
                'joint': joint_for_axis[axis],
            }
            self._limit_switch_prev_triggered_[key] = False
            if device_id not in self._limit_switch_subs_:
                self._limit_switch_can_host_data_[device_id] = None
                self._limit_switch_subs_[device_id] = self.create_subscription(
                    Int32MultiArray, f'serial_rx_{device_id}_unwrapped',
                    lambda msg, did=device_id: self._on_limit_switch_feedback(msg, did), 10)

    def _on_limit_switch_feedback(self, msg: Int32MultiArray, device_id: int):
        self._limit_switch_can_host_data_[device_id] = msg.data

    def _limit_triggered(self, axis, direction):
        """axis('z'/'r')のdirection('lower'/'upper')側リミットスイッチが現在
        トリガーされているか。未配線(device_id=0)・帰還未受信ならFalse。"""
        info = self._limit_switches_.get((axis, direction))
        if info is None:
            return False
        data = self._limit_switch_can_host_data_.get(info['device_id'])
        if data is None:
            return False
        triggered = bool(data[info['slot']] == self.limit_switch_triggered_value_)
        key = (axis, direction)
        if triggered and not self._limit_switch_prev_triggered_.get(key, False):
            self.get_logger().warning(
                f"trajectory_follower_node: {axis}_{direction} limit switch triggered "
                f"(device_id={info['device_id']}, slot={info['slot']}). blocking further "
                f"{'increase' if direction == 'upper' else 'decrease'} of {info['joint']}.")
        self._limit_switch_prev_triggered_[key] = triggered
        return triggered

    def _on_robomas_feedback(self, msg: Int32MultiArray):
        # _on_cubemars_feedbackと同じ理由(起動直後の位置ジャンプ防止)。
        # motor1/motor2の帰還(内蔵ロータエンコーダ基準)からz/rを合成して初期値とする。
        # ただしmotor1/motor2は相対エンコーダのためホーミング未実施の間は無意味な
        # 基準値になる点に注意(robomas_z_offset_m/r_offset_mはhoming_node完了後に
        # SetParametersで反映される。ここでの初期値はホーミング前でも安全な
        # 「現在のロータ位置をそのままz/rの原点とみなす」フォールバックでしかない)。
        #
        # robomas_paused_中(homing_node等がrobomas_device_idを直接制御している間)も
        # 帰還を使ってpos_/target_を追従させ続ける。これをしないと、ホーミング中に
        # 実機が物理的に動いてもpos_/target_はホーミング開始前の値(通常0付近)に
        # 凍結されたままになり、resume_robomas_output時にMITコントローラがその
        # 凍結値へ向けて急激な位置ステップ(=中断すると0付近へ戻ろうとする現象)を
        # 送ってしまう。
        if self.robomas_ is None:
            return
        if self.has_target_ and not self.robomas_paused_:
            return
        cfg = self.robomas_
        m1_deg = msg.data[cfg['motor1_index']] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        m2_deg = msg.data[cfg['motor2_index']] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        m1 = cfg['motor1_sign'] * math.radians(m1_deg) * cfg['pulley_radius_m']
        m2 = cfg['motor2_sign'] * math.radians(m2_deg) * cfg['pulley_radius_m']
        z_offset = self.get_parameter('robomas_z_offset_m').value
        r_offset = self.get_parameter('robomas_r_offset_m').value
        z = cfg['mix_k'] * (m1 + m2) + z_offset
        r = cfg['mix_k'] * (m1 - m2) + r_offset
        for name, val in ((cfg['z_joint'], z), (cfg['r_joint'], r)):
            self.pos_[name] = val
            self.vel_[name] = 0.0
            self.target_[name] = val

    def _on_pause_robomas_output(self, request, response):
        """homing_node等、robomas_device_idへ独立に指令を送る外部ノードのための
        一時停止スイッチ。呼び出している間は_publish_robomas_commands()が
        該当deviceへのpublish自体を丸ごとスキップする(target=0固定送信すらしない。
        homing_node側が明示的に速度指令を出しているため)。"""
        if self.robomas_ is None:
            response.success = False
            response.message = 'robomas_device_id未設定のため対象外です'
            return response
        self.robomas_paused_ = True
        response.success = True
        response.message = 'robomas出力を一時停止しました'
        return response

    def _on_resume_robomas_output(self, request, response):
        if self.robomas_ is None:
            response.success = False
            response.message = 'robomas_device_id未設定のため対象外です'
            return response
        self.robomas_paused_ = False
        response.success = True
        response.message = 'robomas出力を再開しました'
        return response

    def _on_set_parameters(self, params):
        # add_on_set_parameters_callbackに渡されるのはrclpy.parameter.Parameter
        # (Pythonラッパー)であり、.valueはParameterValueメッセージではなく
        # 素のPython値(list等)が直接入っている点に注意。
        n = len(self.joint_names_)
        m = len(self.cubemars_)
        cubemars_array_params = ('cubemars_kp', 'cubemars_kd', 'cubemars_torque_ff')
        robomas_scalar_params = ('robomas_kp', 'robomas_kd', 'robomas_current_ff')
        for p in params:
            if p.name in ('max_velocity', 'max_acceleration') and len(p.value) != n:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must have {n} elements (one per joint_names entry)')
            if p.name in cubemars_array_params and len(p.value) != m:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must have {m} elements (one per cubemars_joint_names entry)')
            if p.name == 'control_mode' and p.value not in VALID_CONTROL_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f'control_mode must be one of {VALID_CONTROL_MODES}')
        for p in params:
            if p.name == 'max_velocity':
                self.max_vel_ = dict(zip(self.joint_names_, p.value))
            elif p.name == 'max_acceleration':
                self.max_accel_ = dict(zip(self.joint_names_, p.value))
            elif p.name == 'control_mode':
                self.control_mode_ = p.value
            elif p.name in cubemars_array_params:
                # cubemars_*系のKp/Kd/torque_ffは_setup_cubemars_outputs()実行時に
                # self.cubemars_[name]へキャッシュされ、_publish_cubemars_commands()は
                # そのキャッシュ値だけを参照する(毎周期get_parameterはしない)ため、
                # set_parametersで変更した値を実際にMIT指令へ反映させるにはここで
                # キャッシュ側も更新する必要がある(2026-08-27、GUIからのゲイン調整用に追加)。
                key = {'cubemars_kp': 'kp', 'cubemars_kd': 'kd',
                       'cubemars_torque_ff': 'torque_ff'}[p.name]
                for name, v in zip(self.cubemars_.keys(), p.value):
                    self.cubemars_[name][key] = float(v)
            elif p.name in robomas_scalar_params and self.robomas_ is not None:
                # robomas_*系も同じ理由(GUI/ros2 param setでのゲイン調整をキャッシュへ反映)。
                key = {'robomas_kp': 'kp', 'robomas_kd': 'kd',
                       'robomas_current_ff': 'current_ff'}[p.name]
                self.robomas_[key] = float(p.value)
        return SetParametersResult(successful=True)

    def target_callback(self, msg: JointState):
        # 送信元はheader.frame_idで判別('auto'=command_gui_node, 'manual'=joy_teleop_node、
        # 未設定は後方互換のため'auto'扱い)。control_modeで受け付ける送信元を絞り込む。
        source = msg.header.frame_id or 'auto'
        if self.control_mode_ == 'auto' and source != 'auto':
            return
        if self.control_mode_ == 'manual' and source != 'manual':
            return

        entries = [(name, pos) for name, pos in zip(msg.name, msg.position) if name in self.target_]
        if not entries:
            return

        # このメッセージに含まれる関節同士を同時到達させる: 各関節の(自分の
        # max_velocity/max_accelerationでの)所要時間のうち最大値に合わせて、
        # 他の関節は速度・加速度を時間軸方向にスケールダウンする。
        times = {name: move_time(pos - self.pos_[name], self.max_vel_[name], self.max_accel_[name])
                 for name, pos in entries}
        t_sync = max(times.values())

        for name, pos in entries:
            self.target_[name] = pos
            scale = (times[name] / t_sync) if t_sync > 1e-9 else 1.0
            self.eff_max_vel_[name] = self.max_vel_[name] * scale
            self.eff_max_accel_[name] = self.max_accel_[name] * (scale ** 2)

        self.has_target_ = True

    def _on_set_root_theta_origin(self, request, response):
        """root_theta_jointのCubeMars本体(AK40-10)へSet Origin(永久原点、
        フラッシュ保存)コマンドを送るようリクエストする。real_joint_bridge_node側の
        root_theta_offset_radは廃止済みのため、この操作が実機の唯一の原点設定手段になる。
        呼び出し前に関節を原点センサの位置(真の機械原点)へ物理的に合わせておくこと。
        """
        name = 'root_theta_joint'
        cfg = self.cubemars_.get(name)
        if cfg is None:
            response.success = False
            response.message = (
                f'{name} はcubemars_joint_names未設定のため実機へSet Originを送信できません')
            return response
        self._origin_pending_[name] = ORIGIN_HOLD_CYCLES
        response.success = True
        response.message = (
            f'{name}: CubeMars(device_id={cfg["device_id"]}, motor_index={cfg["motor_index"]})'
            f'へSet Origin(永久原点)コマンドを送信します')
        return response

    def timer_callback(self):
        if not self.has_target_:
            return

        # z/rはリミットスイッチがトリガーされている方向への移動だけをロックする
        # (反対方向への後退はそのまま許可)。target_自体は書き換えず、この周期の
        # trap_stepに渡す目標だけを現在位置にクランプすることで、スイッチが
        # 外れれば追加のtarget送信なしに自動で元のtarget_へ向けて再開する。
        axis_for_joint = {}
        if self.robomas_ is not None:
            axis_for_joint[self.robomas_['z_joint']] = 'z'
            axis_for_joint[self.robomas_['r_joint']] = 'r'

        for name in self.joint_names_:
            target = self.target_[name]
            axis = axis_for_joint.get(name)
            if axis is not None:
                pos = self.pos_[name]
                if target > pos and self._limit_triggered(axis, 'upper'):
                    target = pos
                elif target < pos and self._limit_triggered(axis, 'lower'):
                    target = pos
            self.pos_[name], self.vel_[name] = trap_step(
                self.pos_[name], self.vel_[name], target,
                self.eff_max_vel_[name], self.eff_max_accel_[name], self.dt_)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self.joint_names_)
        out.position = [self.pos_[name] for name in self.joint_names_]
        self.pub_.publish(out)

        self._publish_cubemars_commands()
        self._publish_robomas_commands()

    def _publish_cubemars_commands(self):
        for name, cfg in self.cubemars_.items():
            m = cfg['motor_index']
            buf = self.device_buffers_[cfg['device_id']]

            remaining = self._origin_pending_.get(name, 0)
            if remaining > 0:
                # Set Origin保持中はMIT指令を送らない(ros2can側cubemars.cppがtarget
                # スロットをorigin_modeとして解釈するため、通常のtarget/kp/kd等は書かない)。
                buf[m] = CUBEMARS_ORIGIN_MODE_PERMANENT
                buf[4 + m] = CUBEMARS_MODE_SET_ORIGIN
                self._origin_pending_[name] = remaining - 1
                continue

            actuator_deg = math.degrees(self.pos_[name]) * cfg['reduction']
            buf[m] = clamp_int16(actuator_deg * 10.0)               # target: 0.1deg/LSB(アクチュエータ軸)
            buf[4 + m] = CUBEMARS_MODE_MIT
            actuator_vel_radps = self.vel_[name] * cfg['reduction']
            buf[8 + m] = clamp_int16(actuator_vel_radps * 100.0)    # mit_velocity: 0.01rad/s/LSB(アクチュエータ軸)
            buf[12 + m] = clamp_int16(cfg['kp'] * 10.0)             # mit_kp: 0.1/LSB
            buf[16 + m] = clamp_int16(cfg['kd'] * 100.0)            # mit_kd: 0.01/LSB
            buf[20 + m] = clamp_int16(cfg['torque_ff'] * 100.0)     # mit_torque_ff: 0.01N・m/LSB

        for device_id, buf in self.device_buffers_.items():
            msg = Int16MultiArray()
            msg.data = list(buf)
            self.device_publishers_[device_id].publish(msg)

    def _publish_robomas_commands(self):
        if self.robomas_ is None or self.robomas_paused_:
            # pause中はhoming_node等の外部ノードがrobomas_device_idを制御している間
            # なので、target=0固定送信すら行わずpublish自体を完全にスキップする。
            return
        cfg = self.robomas_

        z_offset = self.get_parameter('robomas_z_offset_m').value
        r_offset = self.get_parameter('robomas_r_offset_m').value
        # real_joint_bridge_nodeの z = mix_k*(m1+m2) + z_offset_m の逆変換
        # (joint側の値からモータ側の生の変位を求めるため、ここではoffsetを引く)。
        z = self.pos_[cfg['z_joint']] - z_offset
        r = self.pos_[cfg['r_joint']] - r_offset
        z_vel = self.vel_[cfg['z_joint']]
        r_vel = self.vel_[cfg['r_joint']]

        m1 = (z + r) / (2.0 * cfg['mix_k'])
        m2 = (z - r) / (2.0 * cfg['mix_k'])
        m1_vel = (z_vel + r_vel) / (2.0 * cfg['mix_k'])
        m2_vel = (z_vel - r_vel) / (2.0 * cfg['mix_k'])

        m1_deg = cfg['motor1_sign'] * math.degrees(m1 / cfg['pulley_radius_m'])
        m2_deg = cfg['motor2_sign'] * math.degrees(m2 / cfg['pulley_radius_m'])
        m1_rpm = cfg['motor1_sign'] * (m1_vel / cfg['pulley_radius_m']) * (60.0 / (2.0 * math.pi))
        m2_rpm = cfg['motor2_sign'] * (m2_vel / cfg['pulley_radius_m']) * (60.0 / (2.0 * math.pi))

        i1, i2 = cfg['motor1_index'], cfg['motor2_index']
        buf = [0] * ROBOMAS_SLOT_COUNT
        buf[i1] = clamp_int16(m1_deg * 1.0)          # target: 1deg/LSB(アクチュエータ軸)
        buf[i2] = clamp_int16(m2_deg * 1.0)
        buf[4 + i1] = ROBOMAS_MODE_MIT
        buf[4 + i2] = ROBOMAS_MODE_MIT
        buf[8 + i1] = clamp_int16(m1_rpm * 1.0)      # mit_velocity_ff: 1rpm/LSB
        buf[8 + i2] = clamp_int16(m2_rpm * 1.0)
        buf[12 + i1] = clamp_int16(cfg['kp'] * 1000.0)   # mit_kp: 0.001(A/deg)/LSB
        buf[12 + i2] = clamp_int16(cfg['kp'] * 1000.0)
        buf[16 + i1] = clamp_int16(cfg['kd'] * 10000.0)  # mit_kd: 0.0001(A/rpm)/LSB
        buf[16 + i2] = clamp_int16(cfg['kd'] * 10000.0)
        buf[20 + i1] = clamp_int16(cfg['current_ff'] * 1000.0)  # mit_current_ff: 0.001A/LSB
        buf[20 + i2] = clamp_int16(cfg['current_ff'] * 1000.0)

        msg = Int16MultiArray()
        msg.data = buf
        self.robomas_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
