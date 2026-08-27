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
モータ番号(0-3, ros2canのM1-M4に対応)を指定すると、ros2canのトピック規約
(serial_tx_[ID] へInt16MultiArray(24スロット)をpublishすると外部指令として
反映される。ros2can/ros2can/ros_backend.py参照)に従ってMIT指令フレームを
毎周期publishする。device_id/motor_indexの対応は実機配線に依存するため、
未設定(デフォルト)の関節は実機への出力を行わずsoki_simの表示のみ更新する。

MITモードの各スロットのスケールはros2can/firmware/xiao-esp32-s3_can2io/src/
cubemars.cppのコメントと一致させること
  (target: 0.1deg/LSB, mit_velocity: 0.01rad/s/LSB, mit_kp: 0.1/LSB,
   mit_kd: 0.01/LSB, mit_torque_ff: 0.01N・m/LSB)。

/joint_targetsの送信元は自動(command_gui_node)とjoy_teleop_node(手動操作)の
2系統があり、control_modeパラメータ('auto'/'manual'/'both')でどちらを受け付ける
かを切り替えられる(command_gui_nodeの「動作モード」パネルから変更可能)。
送信元の判別にはJointStateメッセージのheader.frame_id('auto'または'manual'、
未設定は'auto'扱い)を使う。台形速度プロファイルによる滑らかな追従はどちらの
送信元でも同じロジックが適用される。
"""
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray

CUBEMARS_MODE_MIT = 2
CUBEMARS_SLOT_COUNT = 24

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
            f'control_mode={self.control_mode_}, cubemars_joints={list(self.cubemars_.keys())})')

    def _setup_cubemars_outputs(self):
        names = list(self.get_parameter('cubemars_joint_names').value)
        device_ids = list(self.get_parameter('cubemars_device_ids').value)
        motor_indices = list(self.get_parameter('cubemars_motor_indices').value)
        kp = list(self.get_parameter('cubemars_kp').value)
        kd = list(self.get_parameter('cubemars_kd').value)
        torque_ff = list(self.get_parameter('cubemars_torque_ff').value)

        lengths = {len(names), len(device_ids), len(motor_indices), len(kp), len(kd), len(torque_ff)}
        if len(names) > 0 and lengths != {len(names)}:
            raise ValueError(
                'cubemars_joint_names/device_ids/motor_indices/kp/kd/torque_ff must all have the same length')
        for name in names:
            if name not in self.joint_names_:
                raise ValueError(f'cubemars_joint_names entry "{name}" is not in joint_names')

        # device_idごとに24スロットのバッファを保持し、対象モータのスロットのみ毎周期更新する
        # (同じdeviceに他モータが同居しない前提。ros2can/note/claude.txtのMODE_CUBEMARS参照)。
        self.cubemars_ = {}
        self.device_buffers_ = {}
        self.device_publishers_ = {}
        for name, device_id, motor_index, kp_v, kd_v, tff_v in zip(
                names, device_ids, motor_indices, kp, kd, torque_ff):
            self.cubemars_[name] = {
                'device_id': int(device_id),
                'motor_index': int(motor_index),
                'kp': float(kp_v),
                'kd': float(kd_v),
                'torque_ff': float(tff_v),
            }
            if device_id not in self.device_buffers_:
                self.device_buffers_[device_id] = [0] * CUBEMARS_SLOT_COUNT
                self.device_publishers_[device_id] = self.create_publisher(
                    Int16MultiArray, f'serial_tx_{device_id}', 10)

    def _on_set_parameters(self, params):
        # add_on_set_parameters_callbackに渡されるのはrclpy.parameter.Parameter
        # (Pythonラッパー)であり、.valueはParameterValueメッセージではなく
        # 素のPython値(list等)が直接入っている点に注意。
        n = len(self.joint_names_)
        for p in params:
            if p.name in ('max_velocity', 'max_acceleration') and len(p.value) != n:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must have {n} elements (one per joint_names entry)')
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

    def timer_callback(self):
        if not self.has_target_:
            return

        for name in self.joint_names_:
            self.pos_[name], self.vel_[name] = trap_step(
                self.pos_[name], self.vel_[name], self.target_[name],
                self.eff_max_vel_[name], self.eff_max_accel_[name], self.dt_)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self.joint_names_)
        out.position = [self.pos_[name] for name in self.joint_names_]
        self.pub_.publish(out)

        self._publish_cubemars_commands()

    def _publish_cubemars_commands(self):
        for name, cfg in self.cubemars_.items():
            m = cfg['motor_index']
            buf = self.device_buffers_[cfg['device_id']]
            pos_deg = math.degrees(self.pos_[name])
            buf[m] = clamp_int16(pos_deg * 10.0)                    # target: 0.1deg/LSB
            buf[4 + m] = CUBEMARS_MODE_MIT
            buf[8 + m] = clamp_int16(self.vel_[name] * 100.0)       # mit_velocity: 0.01rad/s/LSB
            buf[12 + m] = clamp_int16(cfg['kp'] * 10.0)             # mit_kp: 0.1/LSB
            buf[16 + m] = clamp_int16(cfg['kd'] * 100.0)            # mit_kd: 0.01/LSB
            buf[20 + m] = clamp_int16(cfg['torque_ff'] * 100.0)     # mit_torque_ff: 0.01N・m/LSB

        for device_id, buf in self.device_buffers_.items():
            msg = Int16MultiArray()
            msg.data = list(buf)
            self.device_publishers_[device_id].publish(msg)


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
