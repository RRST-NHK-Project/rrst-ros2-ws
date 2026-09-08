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
(2026-08-27追加。_set_cubemars_origin参照。control_mode=3、ros2can/firmware側の
対応実装はcubemars.cpp参照)。呼び出すと数周期(ORIGIN_HOLD_CYCLES)だけ通常のMIT
指令を止めてSET_ORIGINモードを送る。これにより実機エンコーダ自体の原点が電源
off/onを跨いで保持されるため、real_joint_bridge_node側のroot_theta_offset_radに
よるソフトウェア補正は廃止した(note/hardware_mapping.txt参照)。
(tip_theta_jointは2026-09-03に同じSet Origin機構をCubeMars側で使っていたが、
2026-09-08方針変更でROBOMAS(M2006、下記z/rと同じdevice)側の単独直接駆動軸へ
移行した。M2006にSet Origin機構は無く、原点センサも無いため、電源投入前に
機構原点(0deg)へ手で合わせておき、内蔵エンコーダの起動時リセット値をそのまま
原点として使う)。

/joint_targetsの送信元は自動(command_gui_node)とjoy_teleop_node(手動操作)の
2系統があり、control_modeパラメータ('auto'/'manual'/'both')でどちらを受け付ける
かを切り替えられる(command_gui_nodeの「動作モード」パネルから変更可能)。
送信元の判別にはJointStateメッセージのheader.frame_id('auto'または'manual'、
未設定は'auto'扱い)を使う。台形速度プロファイルによる滑らかな追従はどちらの
送信元でも同じロジックが適用される。

z_joint/r_jointについても、実機のロボマス(M2006+C610、motor1/motor2の差動機構、
note/hardware_mapping.txt参照)へMIT(位置PD制御)モードで同時に指令できる
(2026-08-29追加、ros2canのMODE_ROBOMASにMITモードが実装されたことを受けて
root_theta(CubeMars MIT)と対称的に追加)。robomas_device_id等のパラメータで
有効化する(未設定=device_id0のデフォルトのままなら実機出力無効)。z/r(joint角度)
はそれぞれ独立に台形プロファイルされた後、motor_mixer_nodeと同じ式
(m1=(z+r)/(2*mix_k), m2=(z-r)/(2*mix_k))でmotor1/motor2側(アクチュエータ軸)へ
変換してから送信する。位置フィードバックはCubeMars側と異なりロボマス内蔵の
ロータエンコーダを基準に割り切る(z/rの真値である外付けAMTエンコーダとの
バックラッシュ差分は無視する設計判断、note/hardware_mapping.txt参照)。

同じdevice(robomas_device_id)のM3(motor_index=2)にtip_theta_jointを直接駆動で
同居させる(2026-09-08新規、robomas_tip_theta_index等のパラメータで有効化)。
z/rのような差動ミックスは行わず、cubemars_joint_names側のroot_thetaと同様
joint_deg = actuator_deg / tip_theta_reduction の単純な変換のみ。

homing_node(z/rの起動時ホーミング)は本ノードと同じdevice(robomas_device_id)へ
独立に速度指令をpublishするため、ホーミング中に本ノードのMIT指令と衝突する。
これを避けるため、pause_robomas_output/resume_robomas_output(std_srvs/Trigger)
サービスを設け、ホーミング中は本ノード側のrobomas出力を丸ごと止められるように
した(root_theta原点設定の_origin_pending_と同様、外部ノードから能動的に
出力を止めるパターン)。homing_node側がstart_homing_z/start_homing_r時にpause、
stop_homing/完了/タイムアウト失敗時にresumeを呼ぶ。

z/r軸にはそれぞれ上限・下限のリミットスイッチがある(homing_nodeが原点較正に
使う下限側とは別に、上限側は過走防止の安全停止専用、2026-08-31追加)。
本ノードは*_limit_switch_device_id/*_node_index/*_local_indexパラメータ
(z/r × lower/upper の4系統、device_id=0で該当スイッチ無効)でCAN_HOSTの
帰還を直接監視し、timer_callbackでトリガーされている方向への移動だけを
ロックする(反対方向への後退は許可、_limit_triggered参照)。
"""
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16MultiArray, Int32MultiArray
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

# root_theta_jointの内部状態(self.pos_/target_)の起動直後の初期値。0だと
# 「前方=Y+」でフィールドに垂直になるが、フィールドに平行(ハンドがX+側=右側)を
# sim起動時の既定にしたいとのユーザー指定により-90degにする(2026-09-03、
# soki_sim/launch/display.launch.pyのjoint_state_publisher zerosパラメータと
# 一致させること。CubeMars構成時(実機)は最初の目標受信前に_on_cubemars_feedback
# が実機の帰還値で上書きするため、この値は事実上sim専用)。
INITIAL_ROOT_THETA_RAD = -math.pi / 2.0


def clamp_int16(value: float) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(round(value))))


def trap_step(pos: float, vel: float, target: float, max_vel: float, max_accel: float,
              max_decel: float, dt: float):
    """加速度max_accel・減速度max_decel・速度max_velで制限した台形速度プロファイルで
    1ステップ進める。

    残り距離ちょうどでvel=0にできる速度(sqrt(2*max_decel*|error|)、等加速度運動の
    公式 v^2=2ad から導出)を速度上限として使うことで、目標到達時にオーバーシュート
    しないよう自動的に減速する。加速・巡航にはmax_accelを、減速局面
    (|desired_vel| < |vel|、目標に近づいた/目標が現在位置近くへ変わった場合を含む)
    にはmax_decelを使う。同じ加速度で減速も行っていたため、ジョグ停止時の応答が
    鈍かった問題への対策(2026-09-07、ユーザー指摘: 「手動操作のレスポンスが悪い、
    停止時の」)。max_decel>=max_accelにすることで、動き出しはなめらかなまま
    停止だけ素早くできる。
    """
    error = target - pos
    if max_accel <= 0.0 or max_vel <= 0.0 or max_decel <= 0.0:
        return target, 0.0

    brake_vel = math.sqrt(max(0.0, 2.0 * max_decel * abs(error)))
    vel_limit = min(max_vel, brake_vel)
    desired_vel = math.copysign(vel_limit, error) if error != 0.0 else 0.0

    accel_now = max_decel if abs(desired_vel) < abs(vel) else max_accel
    max_dv = accel_now * dt
    dv = max(-max_dv, min(max_dv, desired_vel - vel))
    new_vel = vel + dv
    new_pos = pos + new_vel * dt

    if (error >= 0.0 and new_pos >= target) or (error < 0.0 and new_pos <= target):
        return target, 0.0
    return new_pos, new_vel


def move_time(distance: float, max_vel: float, max_accel: float, max_decel: float) -> float:
    """distance移動するのにtrap_stepの台形プロファイルで要する時間(理論値)。

    加速度max_accel・減速度max_decelが非対称な場合の一般化(max_accel==max_decelの
    ときは従来の対称な台形/三角形の式と一致する)。加速区間+減速区間の距離が
    distance以上なら台形(加速+等速+減速)、届かなければ三角形(加減速のみ)の
    所要時間になる。
    """
    d = abs(distance)
    if d < 1e-9 or max_vel <= 0.0 or max_accel <= 0.0 or max_decel <= 0.0:
        return 0.0
    accel_dist = max_vel * max_vel / (2.0 * max_accel)
    decel_dist = max_vel * max_vel / (2.0 * max_decel)
    if d >= accel_dist + decel_dist:
        cruise_dist = d - accel_dist - decel_dist
        return max_vel / max_accel + max_vel / max_decel + cruise_dist / max_vel
    peak_vel = math.sqrt(2.0 * d * max_accel * max_decel / (max_accel + max_decel))
    return peak_vel / max_accel + peak_vel / max_decel


class TrajectoryFollowerNode(Node):

    def __init__(self):
        super().__init__('trajectory_follower_node')

        self.declare_parameter('joint_names', ['root_theta_joint', 'z_joint', 'r_joint'])
        self.declare_parameter('max_velocity', [1.0, 0.05, 0.05])
        self.declare_parameter('max_acceleration', [2.0, 0.1, 0.1])
        # 減速度(停止時の応答性)。既定はmax_accelerationの2倍(2026-09-07新規、
        # ユーザー指摘: 「手動操作のレスポンスが悪い、停止時の」。動き出しは
        # なめらかなまま停止だけ素早くするため、加速度より大きめにしておく)。
        # joint_namesと同じ要素数が必要(trap_step/move_time参照)。
        self.declare_parameter('max_deceleration', [4.0, 0.2, 0.2])
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('input_topic', 'joint_targets')
        self.declare_parameter('output_topic', 'mixed_joint_states')
        # 'auto'=command_gui_nodeのみ受付, 'manual'=joy_teleop_nodeのみ受付, 'both'=両方受付
        self.declare_parameter('control_mode', 'auto')
        # 組立中・不具合時に特定の軸を無視するためのランタイム設定(2026-09-05追加、
        # command_gui_nodeの「軸の有効/無効」パネルから変更する)。ここに含まれる
        # joint_namesはtarget_callbackで目標更新を無視し、現在位置で凍結される
        # (モータへのMIT指令自体は現在位置保持として送り続ける。z_joint/r_jointは
        # ロボマス差動のため、片方だけ無効化してももう片方は通常通り動く一方、
        # 両モータへの指令自体は継続する点に注意)。空配列(デフォルト)ならどの
        # 軸も無効化しない。yamlには保存しないランタイム専用値。
        self.declare_parameter('disabled_joints', [], ParameterDescriptor(dynamic_typing=True))

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
        # 動き出しキック(静止摩擦補償、2026-09-09追加)。motor1/motor2それぞれの
        # 台形プロファイル速度(vel_、m1_vel/m2_vel換算後)が0から動き出した瞬間から
        # robomas_kick_duration_secの間だけ、robomas_kick_current_aをその方向
        # (速度の符号)に応じた符号で電流指令へ上乗せする(current_ffと違い方向に
        # 応じて符号が変わるため、逆方向の動きを阻害しない。ユーザー報告:
        # 「差動のロボマスが片方だけ周りはじめのトルクが足りない」への対応。
        # current_ffは方向非依存の固定符号なので今回のような動き出しの摩擦補償には
        # 不向きと判断し、代わりにこちらを追加した。_compute_kick_current参照)。
        # robomas_kick_enabled=falseなら既存動作のまま完全に無効(既定false)。
        self.declare_parameter('robomas_kick_enabled', False)
        self.declare_parameter('robomas_kick_current_a', 0.1)
        self.declare_parameter('robomas_kick_duration_sec', 0.05)
        self.declare_parameter('robomas_kick_vel_threshold_mps', 0.001)
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

        # ROBOMAS M3 (tip_theta_joint、2026-09-08新規)。z/rのような差動ミックスでは
        # なく単独直接駆動(joint_deg = actuator_deg / tip_theta_reduction)。
        # robomas_tip_theta_index未設定(既定-1)ならtip_theta側の実機出力は無効
        # (root_theta_reduction等と同じ「未設定なら出力無効」パターン)。原点センサが
        # 無いため、電源投入前に機構原点(0deg)へ手で合わせておく前提で、M2006内蔵
        # エンコーダの起動時リセット値(=0)をそのまま原点として扱う(ホーミング無し)。
        self.declare_parameter('robomas_tip_theta_index', -1)   # M3
        self.declare_parameter('robomas_tip_theta_joint', 'tip_theta_joint')
        self.declare_parameter('tip_theta_reduction', 1.4)      # 28T/20T
        self.declare_parameter('tip_theta_sign', 1.0)
        self.declare_parameter('tip_theta_offset_rad', 0.0)     # 通常0(手動ゼロ合わせ前提)
        self.declare_parameter('robomas_tip_theta_kp', 0.0)
        self.declare_parameter('robomas_tip_theta_kd', 0.0)
        self.declare_parameter('robomas_tip_theta_current_ff', 0.0)

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
        max_decel = list(self.get_parameter('max_deceleration').value)
        if (len(max_vel) != len(self.joint_names_) or len(max_accel) != len(self.joint_names_)
                or len(max_decel) != len(self.joint_names_)):
            raise ValueError(
                'max_velocity/max_acceleration/max_deceleration must have the same length as joint_names')

        self.control_mode_ = self.get_parameter('control_mode').value
        if self.control_mode_ not in VALID_CONTROL_MODES:
            raise ValueError(f'control_mode must be one of {VALID_CONTROL_MODES}')

        self.disabled_joints_ = set(self.get_parameter('disabled_joints').value)
        if any(name not in self.joint_names_ for name in self.disabled_joints_):
            raise ValueError('disabled_joints entries must be in joint_names')

        self.max_vel_ = dict(zip(self.joint_names_, max_vel))
        self.max_accel_ = dict(zip(self.joint_names_, max_accel))
        self.max_decel_ = dict(zip(self.joint_names_, max_decel))
        # 同時到達スケーリング後の実効値。target_callbackで毎回更新される
        # (未受信時はmax_vel_/max_accel_/max_decel_と同じ=通常の単軸プロファイル)。
        self.eff_max_vel_ = dict(self.max_vel_)
        self.eff_max_accel_ = dict(self.max_accel_)
        self.eff_max_decel_ = dict(self.max_decel_)
        self.pos_ = {name: 0.0 for name in self.joint_names_}
        if 'root_theta_joint' in self.pos_:
            self.pos_['root_theta_joint'] = INITIAL_ROOT_THETA_RAD
        self.vel_ = {name: 0.0 for name in self.joint_names_}
        self.target_ = dict(self.pos_)
        self.has_target_ = False

        self._setup_cubemars_outputs()
        self._setup_robomas_outputs()
        self._setup_limit_switches()
        self.create_service(Trigger, 'set_root_theta_origin', self._on_set_root_theta_origin)

        # ---- ソフト緊急停止 (2026-09-08追加、CAN_HOST(device_id=101)実機の赤色状態
        # 表示灯の「点滅(速)」に対応。engage_estop中はtimer_callbackが早い段階で
        # returnするため、trap_stepの進行もcubemars/robomas双方への指令publishも完全に
        # 止まる(pause_robomas_output/resume_robomas_outputと同じ「publish自体を
        # スキップする」方式。実機側は最後に受け取ったMIT指令をそのまま保持し続ける
        # ため、現在位置で静止する)。command_gui_node側がホーミングの中断・
        # 自動シーケンスの中断と合わせて呼び出す想定(単体ではhoming_nodeの動作は
        # 止まらない、そちらはstop_homingを別途呼ぶ必要がある)。 ----
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estop_active_ = False
        self.estop_pub_ = self.create_publisher(Bool, 'estop_active', latched_qos)
        self._publish_estop_state()
        self.create_service(Trigger, 'engage_estop', self._on_engage_estop)
        self.create_service(Trigger, 'release_estop', self._on_release_estop)

        # ---- リミットスイッチによる安全停止中フラグ (2026-09-08追加、赤色状態表示灯の
        # 「点灯」に対応。z/r上限・下限いずれか1個でもトリガーされていればTrueとする
        # (どちらの方向への移動をロックしているかまでは区別しない、GUI表示用の集約値)。----
        self.limit_stop_active_ = False
        self.limit_stop_pub_ = self.create_publisher(Bool, 'limit_stop_active', latched_qos)
        self._publish_limit_stop_state()

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
            f'limit_switches={sorted("_".join(k) for k in self._limit_switches_.keys())}, '
            f'disabled_joints={sorted(self.disabled_joints_)})')

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
        # 最新のm1/m2(motor1_joint/motor2_joint、pulley_radius_m換算済み)。
        # robomas_z_offset_m/r_offset_m変更時にpos_/target_を同期的に再計算する
        # 用(_on_robomas_feedback/_on_set_parameters参照)。帰還未受信ならNone。
        self._last_robomas_m1_m2_ = None
        # 動き出しキック(_compute_kick_current参照)のモータ別状態。1=motor1,
        # 2=motor2。was_moving: 前回周期で「動いていた」か。kick_start_time:
        # 直近で停止->動き出しへ遷移したmonotonic時刻(未遷移ならNone)。
        self._robomas_kick_was_moving_ = {1: False, 2: False}
        self._robomas_kick_start_time_ = {1: None, 2: None}
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
            'kick_enabled': bool(self.get_parameter('robomas_kick_enabled').value),
            'kick_current_a': float(self.get_parameter('robomas_kick_current_a').value),
            'kick_duration_sec': float(self.get_parameter('robomas_kick_duration_sec').value),
            'kick_vel_threshold_mps': float(self.get_parameter('robomas_kick_vel_threshold_mps').value),
            'z_joint': z_name,
            'r_joint': r_name,
            'mix_k': float(self.get_parameter('robomas_mix_k').value),
            'pulley_radius_m': (float(self.get_parameter('robomas_pulley_pitch_diameter_mm').value)
                                 / 2.0 / 1000.0),
            'motor1_sign': float(self.get_parameter('robomas_motor1_sign').value),
            'motor2_sign': float(self.get_parameter('robomas_motor2_sign').value),
        }
        tip_theta_index = int(self.get_parameter('robomas_tip_theta_index').value)
        if tip_theta_index >= 0:
            tip_theta_name = self.get_parameter('robomas_tip_theta_joint').value
            if tip_theta_name not in self.joint_names_:
                raise ValueError('robomas_tip_theta_joint must be in joint_names')
            self.robomas_['tip_theta'] = {
                'motor_index': tip_theta_index,
                'joint': tip_theta_name,
                'reduction': float(self.get_parameter('tip_theta_reduction').value),
                'sign': float(self.get_parameter('tip_theta_sign').value),
                'offset': float(self.get_parameter('tip_theta_offset_rad').value),
                'kp': float(self.get_parameter('robomas_tip_theta_kp').value),
                'kd': float(self.get_parameter('robomas_tip_theta_kd').value),
                'current_ff': float(self.get_parameter('robomas_tip_theta_current_ff').value),
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
        cfg = self.robomas_
        m1_deg = msg.data[cfg['motor1_index']] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        m2_deg = msg.data[cfg['motor2_index']] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        m1 = cfg['motor1_sign'] * math.radians(m1_deg) * cfg['pulley_radius_m']
        m2 = cfg['motor2_sign'] * math.radians(m2_deg) * cfg['pulley_radius_m']
        # robomas_z_offset_m/r_offset_mが変わった瞬間にpos_/target_を同期的に
        # 再計算するため(_on_set_parameters参照)、has_target_/robomas_paused_の
        # 状態に関わらず常に最新のm1/m2をキャッシュしておく。
        self._last_robomas_m1_m2_ = (m1, m2)
        if self.has_target_ and not self.robomas_paused_:
            return
        z_offset = self.get_parameter('robomas_z_offset_m').value
        r_offset = self.get_parameter('robomas_r_offset_m').value
        z = cfg['mix_k'] * (m1 + m2) + z_offset
        r = cfg['mix_k'] * (m1 - m2) + r_offset
        for name, val in ((cfg['z_joint'], z), (cfg['r_joint'], r)):
            self.pos_[name] = val
            self.vel_[name] = 0.0
            self.target_[name] = val

        # tip_theta(M3)はz/rの差動ミックスとは独立した単独直接駆動
        # (_publish_robomas_commandsのtip_theta側と対称、2026-09-08新規)。
        tip_cfg = cfg.get('tip_theta')
        if tip_cfg is not None:
            tip_deg = msg.data[tip_cfg['motor_index']] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
            tip_theta = (tip_cfg['sign'] * math.radians(tip_deg) / tip_cfg['reduction']
                         + tip_cfg['offset'])
            self.pos_[tip_cfg['joint']] = tip_theta
            self.vel_[tip_cfg['joint']] = 0.0
            self.target_[tip_cfg['joint']] = tip_theta

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

    def _publish_estop_state(self):
        msg = Bool()
        msg.data = self.estop_active_
        self.estop_pub_.publish(msg)

    def _on_engage_estop(self, request, response):
        self.estop_active_ = True
        self._publish_estop_state()
        self.get_logger().warning(
            'trajectory_follower_node: emergency stop engaged, cubemars/robomas output frozen')
        response.success = True
        response.message = 'emergency stop engaged'
        return response

    def _on_release_estop(self, request, response):
        self.estop_active_ = False
        self._publish_estop_state()
        self.get_logger().warning('trajectory_follower_node: emergency stop released')
        response.success = True
        response.message = 'emergency stop released'
        return response

    def _publish_limit_stop_state(self):
        msg = Bool()
        msg.data = self.limit_stop_active_
        self.limit_stop_pub_.publish(msg)

    def _update_limit_stop_status(self):
        active = any(self._limit_triggered(axis, direction)
                     for axis, direction in self._limit_switches_.keys())
        if active != self.limit_stop_active_:
            self.limit_stop_active_ = active
            self._publish_limit_stop_state()

    def _on_set_parameters(self, params):
        # add_on_set_parameters_callbackに渡されるのはrclpy.parameter.Parameter
        # (Pythonラッパー)であり、.valueはParameterValueメッセージではなく
        # 素のPython値(list等)が直接入っている点に注意。
        n = len(self.joint_names_)
        m = len(self.cubemars_)
        cubemars_array_params = ('cubemars_kp', 'cubemars_kd', 'cubemars_torque_ff')
        robomas_scalar_params = ('robomas_kp', 'robomas_kd', 'robomas_current_ff')
        robomas_tip_theta_params = ('robomas_tip_theta_kp', 'robomas_tip_theta_kd',
                                     'robomas_tip_theta_current_ff')
        for p in params:
            if p.name in ('max_velocity', 'max_acceleration', 'max_deceleration') and len(p.value) != n:
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
            if p.name == 'disabled_joints' and any(name not in self.joint_names_ for name in p.value):
                return SetParametersResult(
                    successful=False,
                    reason='disabled_joints entries must be in joint_names')
        for p in params:
            if p.name == 'max_velocity':
                self.max_vel_ = dict(zip(self.joint_names_, p.value))
            elif p.name == 'max_acceleration':
                self.max_accel_ = dict(zip(self.joint_names_, p.value))
            elif p.name == 'max_deceleration':
                self.max_decel_ = dict(zip(self.joint_names_, p.value))
            elif p.name == 'control_mode':
                self.control_mode_ = p.value
            elif p.name == 'disabled_joints':
                # 新たに無効化された軸は、以後target_callbackが目標更新を無視するだけ
                # でなく、ここで即座にtarget_を現在位置へ固定して移動中なら滑らかに
                # 停止させる(trap_stepがerror=0として台形プロファイルで減速する)。
                new_disabled = set(p.value)
                for name in new_disabled - self.disabled_joints_:
                    self.target_[name] = self.pos_[name]
                self.disabled_joints_ = new_disabled
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
            elif (p.name in robomas_tip_theta_params and self.robomas_ is not None
                  and 'tip_theta' in self.robomas_):
                # tip_theta(M3)はz/rと別ゲインを持つため、robomas_scalar_paramsとは
                # 別にself.robomas_['tip_theta']側のキャッシュを更新する。
                key = {'robomas_tip_theta_kp': 'kp', 'robomas_tip_theta_kd': 'kd',
                       'robomas_tip_theta_current_ff': 'current_ff'}[p.name]
                self.robomas_['tip_theta'][key] = float(p.value)
            elif p.name == 'robomas_kick_enabled' and self.robomas_ is not None:
                self.robomas_['kick_enabled'] = bool(p.value)
            elif (p.name in ('robomas_kick_current_a', 'robomas_kick_duration_sec',
                              'robomas_kick_vel_threshold_mps')
                  and self.robomas_ is not None):
                key = {'robomas_kick_current_a': 'kick_current_a',
                       'robomas_kick_duration_sec': 'kick_duration_sec',
                       'robomas_kick_vel_threshold_mps': 'kick_vel_threshold_mps'}[p.name]
                self.robomas_[key] = float(p.value)
            elif (p.name in ('robomas_z_offset_m', 'robomas_r_offset_m')
                  and self.robomas_ is not None and self._last_robomas_m1_m2_ is not None):
                # homing_node完了時にhoming_nodeから送られてくる(_apply_offset参照)。
                # offsetの変更は「現在の実位置の基準を較正し直す」操作でしかない
                # はずだが、pos_/target_の再計算を_on_robomas_feedbackの次回呼び出し
                # 任せにすると、それがいつ来るか(robomas_paused_解除やhas_target_との
                # 兼ね合い、executorのスケジューリング順序)に依存してしまい、古い
                # 基準のままのpos_/target_へ向けてMIT指令が「唐突に動き出す」ことが
                # あった(2026-09-08、ユーザー報告: 「ホーミング終了後に目標値に
                # 向かっているならやめてほしい。その位置で停止してもらって構わない」)。
                # そこでoffsetパラメータが変わったこのコールバックの中で同期的に
                # pos_/target_を再計算し、そのまま静止させる(_on_robomas_feedbackの
                # z/r計算と同じ式。z/rは差動ミックスなので、片方のoffsetだけが今回の
                # paramsに含まれていてももう片方も一緒に再計算しておく)。
                cfg = self.robomas_
                m1, m2 = self._last_robomas_m1_m2_
                z_offset = (float(p.value) if p.name == 'robomas_z_offset_m'
                            else self.get_parameter('robomas_z_offset_m').value)
                r_offset = (float(p.value) if p.name == 'robomas_r_offset_m'
                            else self.get_parameter('robomas_r_offset_m').value)
                z = cfg['mix_k'] * (m1 + m2) + z_offset
                r = cfg['mix_k'] * (m1 - m2) + r_offset
                for name, val in ((cfg['z_joint'], z), (cfg['r_joint'], r)):
                    self.pos_[name] = val
                    self.vel_[name] = 0.0
                    self.target_[name] = val
        return SetParametersResult(successful=True)

    def target_callback(self, msg: JointState):
        # 送信元はheader.frame_idで判別('auto'=command_gui_node, 'manual'=joy_teleop_node、
        # 未設定は後方互換のため'auto'扱い)。control_modeで受け付ける送信元を絞り込む。
        source = msg.header.frame_id or 'auto'
        if self.control_mode_ == 'auto' and source != 'auto':
            return
        if self.control_mode_ == 'manual' and source != 'manual':
            return

        entries = [(name, pos) for name, pos in zip(msg.name, msg.position)
                   if name in self.target_ and name not in self.disabled_joints_]
        if not entries:
            return

        # このメッセージに含まれる関節同士を同時到達させる: 各関節の(自分の
        # max_velocity/max_accelerationでの)所要時間のうち最大値に合わせて、
        # 他の関節は速度・加速度を時間軸方向にスケールダウンする。
        times = {name: move_time(pos - self.pos_[name], self.max_vel_[name], self.max_accel_[name],
                                  self.max_decel_[name])
                 for name, pos in entries}
        t_sync = max(times.values())

        for name, pos in entries:
            self.target_[name] = pos
            scale = (times[name] / t_sync) if t_sync > 1e-9 else 1.0
            self.eff_max_vel_[name] = self.max_vel_[name] * scale
            self.eff_max_accel_[name] = self.max_accel_[name] * (scale ** 2)
            self.eff_max_decel_[name] = self.max_decel_[name] * (scale ** 2)

        self.has_target_ = True

    def _set_cubemars_origin(self, name, response):
        """nameのCubeMars本体(AK40-10)へSet Origin(永久原点、フラッシュ保存)
        コマンドを送るようリクエストする(_on_set_root_theta_origin参照)。
        呼び出し前に関節を原点センサの位置(真の機械原点)へ物理的に合わせておくこと。
        (2026-09-08方針変更: tip_theta_jointはROBOMAS(M2006)側へ移行し、Set Origin
        機能自体を持たないため対象外になった。原点センサも無く、電源投入前の手動
        ゼロ合わせ+起動時リセットされる内蔵エンコーダの値をそのまま原点として使う)
        """
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

    def _on_set_root_theta_origin(self, request, response):
        # real_joint_bridge_node側のroot_theta_offset_radは廃止済みのため、
        # この操作が実機の唯一の原点設定手段になる(_set_cubemars_origin参照)。
        return self._set_cubemars_origin('root_theta_joint', response)

    def timer_callback(self):
        if not self.has_target_:
            return

        self._update_limit_stop_status()
        if self.estop_active_:
            # 緊急停止中はtrap_stepの進行・cubemars/robomasへのpublishを一切行わない
            # (最後に送った指令のまま実機側で位置保持される、engage_estop参照)。
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
                self.eff_max_vel_[name], self.eff_max_accel_[name], self.eff_max_decel_[name], self.dt_)

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

    def _compute_kick_current(self, motor_key, vel_signed_mps, cfg):
        """動き出しキック(静止摩擦補償)の電流[A]を返す(2026-09-08追加、ファイル
        冒頭のrobomas_kick_*パラメータ宣言部コメント参照)。vel_signed_mpsは実際に
        モータへ送る電流の符号と揃えた速度(motor1_sign/motor2_sign適用済み、
        _publish_robomas_commands参照)。motor_keyは1または2で、motor1/motor2
        それぞれ独立に「止まっていた(|vel|<=閾値)状態から動き出した」瞬間を
        検出し、そこからkick_duration_secの間だけkick_current_aをvel_signed_mpsと
        同じ符号で返す(current_ffと違い移動方向に応じて符号が変わるため、
        逆方向の動きを阻害しない)。robomas_kick_enabled=falseなら常に0.0を返し、
        内部状態もリセットする(無効化時は既存動作(current_ffのみ)に戻る)。"""
        if not cfg['kick_enabled']:
            self._robomas_kick_was_moving_[motor_key] = False
            self._robomas_kick_start_time_[motor_key] = None
            return 0.0

        moving = abs(vel_signed_mps) > cfg['kick_vel_threshold_mps']
        now = time.monotonic()
        if moving and not self._robomas_kick_was_moving_[motor_key]:
            self._robomas_kick_start_time_[motor_key] = now
        self._robomas_kick_was_moving_[motor_key] = moving
        if not moving:
            return 0.0

        start = self._robomas_kick_start_time_[motor_key]
        if start is None or (now - start) >= cfg['kick_duration_sec']:
            return 0.0
        return cfg['kick_current_a'] if vel_signed_mps > 0.0 else -cfg['kick_current_a']

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

        # 動き出しキック(静止摩擦補償)。motor1_sign/motor2_signを適用済みの
        # 符号付き速度(実際にモータへ流す電流の符号と揃える)で、モータごとに
        # 独立して停止->動き出しの遷移を検出する(_compute_kick_current参照)。
        m1_signed_vel = cfg['motor1_sign'] * m1_vel
        m2_signed_vel = cfg['motor2_sign'] * m2_vel
        kick1 = self._compute_kick_current(1, m1_signed_vel, cfg)
        kick2 = self._compute_kick_current(2, m2_signed_vel, cfg)

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
        buf[20 + i1] = clamp_int16((cfg['current_ff'] + kick1) * 1000.0)  # mit_current_ff: 0.001A/LSB
        buf[20 + i2] = clamp_int16((cfg['current_ff'] + kick2) * 1000.0)

        # tip_theta(M3)はz/rの差動ミックスとは独立した単独直接駆動
        # (_publish_cubemars_commandsのroot_theta単軸パターンと同型、2026-09-08新規)。
        tip_cfg = cfg.get('tip_theta')
        if tip_cfg is not None:
            i3 = tip_cfg['motor_index']
            tip_pos = self.pos_[tip_cfg['joint']]
            tip_vel = self.vel_[tip_cfg['joint']]
            tip_deg = tip_cfg['sign'] * math.degrees((tip_pos - tip_cfg['offset']) * tip_cfg['reduction'])
            tip_rpm = tip_cfg['sign'] * (tip_vel * tip_cfg['reduction']) * (60.0 / (2.0 * math.pi))
            buf[i3] = clamp_int16(tip_deg * 1.0)             # target: 1deg/LSB(アクチュエータ軸)
            buf[4 + i3] = ROBOMAS_MODE_MIT
            buf[8 + i3] = clamp_int16(tip_rpm * 1.0)         # mit_velocity_ff: 1rpm/LSB
            buf[12 + i3] = clamp_int16(tip_cfg['kp'] * 1000.0)     # mit_kp: 0.001(A/deg)/LSB
            buf[16 + i3] = clamp_int16(tip_cfg['kd'] * 10000.0)    # mit_kd: 0.0001(A/rpm)/LSB
            buf[20 + i3] = clamp_int16(tip_cfg['current_ff'] * 1000.0)  # mit_current_ff: 0.001A/LSB

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
