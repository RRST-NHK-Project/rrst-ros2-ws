#!/usr/bin/env python3
"""z_joint/r_joint(motor1/motor2差動、M2006+C610)の起動時ホーミングノード。

motor1/motor2は外付けAMTエンコーダ(ENC1/ENC2、CAN_HOST配下ノード)による
相対(インクリメンタル)エンコーダのため、電源投入毎に基準を喪失する
(詳細はnote/hardware_mapping.txt、note/can_mapping.txt参照)。

原点センサはz軸・r軸それぞれに1個ずつ(motor1/motor2の個別軸ではない、
soki本体で確認済み)。ただし z = mix_k*(m1+m2), r = mix_k*(m1-m2) という
合成後の量なので、ros2canの/zero_channel(ENC1またはENC2を単体でゼロ化する
だけの機能)では正しく較正できない。そこで本ノードは:

  1. motor1/motor2を同方向に駆動してz軸の原点センサに当たるまで動かす
     (rが変化しないよう2モータを同じ向きに動かす)
  2. センサ検出時点の生ENC値から z_raw を計算し、z_offset_m = z_ref_value_m -
     z_raw を求める
  3. 続けてmotor1/motor2を差動方向に駆動してr軸の原点センサに当たるまで動かす
     (zが変化しないよう2モータを逆向きに動かす)
  4. センサ検出時点の生ENC値から r_raw を計算し、r_offset_m = r_ref_value_m -
     r_raw を求める
  5. z_offset_m/r_offset_mをreal_joint_bridge_node・trajectory_follower_nodeの
     両方へSetParametersサービスで反映する(前者はsim表示用、後者は実機への
     MIT指令生成用。どちらもyamlではなくランタイムパラメータとして保持する設計。
     詳細はreal_joint_bridge_node.py冒頭コメント・trajectory_follower_node.pyの
     robomas_z_offset_m/r_offset_m参照)

安全のため、ノード起動時に自動でホーミングは開始しない
(`start_homing`(std_srvs/Trigger)サービスを明示的に呼んだときのみ開始)。
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
(2026-08-29追加)。そこで本ノードはstart_homing時にtrajectory_follower_nodeの
pause_robomas_output(std_srvs/Trigger)を呼んで出力を止め、stop_homing・
ホーミング完了・タイムアウト失敗のいずれの場合もresume_robomas_outputを呼んで
再開させる。trajectory_follower_node未起動(実機出力無効)でもservice呼び出しが
失敗するだけで安全(ログ警告のみ、ホーミング自体は継続)。

ros2can側の前提:
  - ROBOMAS device(motor1/motor2の速度指令)の`topic_passthrough`をGUIで
    ONにしておかないと、本ノードの/serial_tx_[id]への指令が反映されない。
  - CAN_HOST device配下ノードのSW1/SW2をz軸/r軸の原点センサ入力として
    割り当てている想定(note/can_mapping.txt「z/r原点センサ(ホーミング用)」
    参照、未確認なら要修正)。
  - モータ回転方向とz/r方向の対応(*_home_motor*_vel_sign)、原点センサ検出時の
    真値(z_ref_value_m/r_ref_value_m)は実測が必要。実機で少しずつ検証しながら
    調整すること。低速(デフォルト30rpm)・タイムアウト(デフォルト20秒)を
    必ず設定した状態で試すこと。
"""

import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

STATE_IDLE = 'idle'
STATE_HOMING_Z = 'homing_z'
STATE_HOMING_R = 'homing_r'
STATE_DONE = 'done'
STATE_FAILED = 'failed'

SLOT_COUNT = 24


class HomingNode(Node):

    def __init__(self):
        super().__init__('homing_node')

        # ---- CAN_HOST配下ノードのENC1/ENC2・SW1/SW2 (motor1/motor2, z/r原点センサ) ----
        self.declare_parameter('can_host_device_id', 101)
        self.declare_parameter('can_host_node_index', 1)
        self.declare_parameter('can_host_slots_per_node', 5)
        self.declare_parameter('enc1_local_index', 3)
        self.declare_parameter('enc2_local_index', 4)
        self.declare_parameter('enc1_is_motor1', True)
        self.declare_parameter('motor_encoder_counts_per_rev', 2048.0)
        self.declare_parameter('pulley_pitch_diameter_mm', 22.92)
        self.declare_parameter('mix_k', 0.5)
        self.declare_parameter('motor1_sign', 1.0)
        self.declare_parameter('motor2_sign', 1.0)
        # 要確認(note/can_mapping.txt「z/r原点センサ(ホーミング用)」)。
        # 現状の仮定: z軸原点センサ=SW1、r軸原点センサ=SW2、どちらもENC1/ENC2と
        # 同じノード(node_index)に載っている。
        self.declare_parameter('z_limit_switch_node_index', 1)
        self.declare_parameter('z_limit_switch_local_index', 0)   # SW1
        self.declare_parameter('r_limit_switch_node_index', 1)
        self.declare_parameter('r_limit_switch_local_index', 1)   # SW2
        self.declare_parameter('switch_triggered_value', 1)       # !digitalRead()相当

        # ---- ROBOMAS device(motor1/motor2の速度指令) ----
        self.declare_parameter('robomas_device_id', 21)
        self.declare_parameter('robomas_motor1_slot', 0)   # M1 target_velocity
        self.declare_parameter('robomas_motor2_slot', 1)   # M2 target_velocity

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
        self.can_host_node_index_ = gp('can_host_node_index').value
        self.can_host_slots_per_node_ = gp('can_host_slots_per_node').value
        self.enc1_local_index_ = gp('enc1_local_index').value
        self.enc2_local_index_ = gp('enc2_local_index').value
        self.enc1_is_motor1_ = gp('enc1_is_motor1').value
        self.motor_cpr_ = gp('motor_encoder_counts_per_rev').value
        self.pulley_radius_m_ = (gp('pulley_pitch_diameter_mm').value / 2.0) / 1000.0
        self.mix_k_ = gp('mix_k').value
        self.motor1_sign_ = gp('motor1_sign').value
        self.motor2_sign_ = gp('motor2_sign').value

        self.z_sw_node_index_ = gp('z_limit_switch_node_index').value
        self.z_sw_local_index_ = gp('z_limit_switch_local_index').value
        self.r_sw_node_index_ = gp('r_limit_switch_node_index').value
        self.r_sw_local_index_ = gp('r_limit_switch_local_index').value
        self.switch_triggered_value_ = gp('switch_triggered_value').value

        self.robomas_device_id_ = gp('robomas_device_id').value
        self.robomas_motor1_slot_ = gp('robomas_motor1_slot').value
        self.robomas_motor2_slot_ = gp('robomas_motor2_slot').value

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

        self.enc1_slot_ = self.can_host_node_index_ * self.can_host_slots_per_node_ + self.enc1_local_index_
        self.enc2_slot_ = self.can_host_node_index_ * self.can_host_slots_per_node_ + self.enc2_local_index_
        self.z_sw_slot_ = self.z_sw_node_index_ * self.can_host_slots_per_node_ + self.z_sw_local_index_
        self.r_sw_slot_ = self.r_sw_node_index_ * self.can_host_slots_per_node_ + self.r_sw_local_index_

        self.can_host_data_ = None
        self.state_ = STATE_IDLE
        self.phase_start_time_ = None
        self.z_offset_m_ = None

        self.create_subscription(
            Int32MultiArray, f'serial_rx_{self.can_host_device_id_}_unwrapped',
            self._on_can_host, 10)
        self.tx_pub_ = self.create_publisher(
            Int16MultiArray, f'serial_tx_{self.robomas_device_id_}', 10)

        self._set_params_cli = self.create_client(
            SetParameters, f'/{bridge_node_name}/set_parameters')
        self._set_params_cli_traj_ = self.create_client(
            SetParameters, f'/{trajectory_follower_node_name}/set_parameters')
        self._pause_robomas_cli_ = self.create_client(Trigger, 'pause_robomas_output')
        self._resume_robomas_cli_ = self.create_client(Trigger, 'resume_robomas_output')

        self.create_service(Trigger, 'start_homing', self._on_start_homing)
        self.create_service(Trigger, 'stop_homing', self._on_stop_homing)
        self.create_service(Trigger, 'skip_homing', self._on_skip_homing)

        self.create_timer(control_period, self._on_tick)

        self.get_logger().info(
            f'homing_node started (idle): ENC1=slot{self.enc1_slot_}, ENC2=slot{self.enc2_slot_}, '
            f'z_sw=slot{self.z_sw_slot_}, r_sw=slot{self.r_sw_slot_}, '
            f'robomas device_id={self.robomas_device_id_}. '
            f"call 'start_homing' service to begin.")

    # ---------------- サービス ----------------

    def _on_start_homing(self, request, response):
        if self.state_ in (STATE_HOMING_Z, STATE_HOMING_R):
            response.success = False
            response.message = f'already running (state={self.state_})'
            return response
        if self.can_host_data_ is None:
            response.success = False
            response.message = 'serial_rx_*_unwrapped not received yet'
            return response
        self.state_ = STATE_HOMING_Z
        self.phase_start_time_ = time.monotonic()
        self.z_offset_m_ = None
        self._call_trigger_async(self._pause_robomas_cli_, 'pause_robomas_output')
        self.get_logger().info('homing_node: start (phase=z)')
        response.success = True
        response.message = 'homing started (phase=z)'
        return response

    def _on_stop_homing(self, request, response):
        self._send_velocity(0.0, 0.0)
        self.state_ = STATE_IDLE
        self._call_trigger_async(self._resume_robomas_cli_, 'resume_robomas_output')
        self.get_logger().warning('homing_node: stopped by request')
        response.success = True
        response.message = 'stopped'
        return response

    def _on_skip_homing(self, request, response):
        """motor1/motor2を駆動せず、現在位置をz_ref_value_m/r_ref_value_mの位置と
        みなしてoffsetを即座に確定する(ファイル冒頭docstring参照)。呼び出し前に
        機体を原点センサ位置相当へ物理的に合わせておくこと。"""
        if self.state_ in (STATE_HOMING_Z, STATE_HOMING_R):
            response.success = False
            response.message = f'ホーミング動作中は使えません(state={self.state_})'
            return response
        if self.can_host_data_ is None:
            response.success = False
            response.message = 'serial_rx_*_unwrapped not received yet'
            return response

        m1, m2 = self._current_motor_joints()
        z_raw = self.mix_k_ * (m1 + m2)
        r_raw = self.mix_k_ * (m1 - m2)
        z_offset_m = self.z_ref_value_m_ - z_raw
        r_offset_m = self.r_ref_value_m_ - r_raw
        self._apply_offsets(z_offset_m, r_offset_m)
        self.state_ = STATE_DONE
        self.get_logger().warning(
            f'homing_node: skip_homing実行(モータ駆動なし)。現在位置を'
            f'z={self.z_ref_value_m_:.5f}, r={self.r_ref_value_m_:.5f} とみなしました '
            f'(z_offset_m={z_offset_m:.5f}, r_offset_m={r_offset_m:.5f})')
        response.success = True
        response.message = (
            f'skip_homing: z_offset_m={z_offset_m:.5f}, r_offset_m={r_offset_m:.5f} '
            f'を反映しました(機体が正しい位置にあった前提)')
        return response

    def _call_trigger_async(self, client, label):
        # trajectory_follower_node未起動(実機出力無効)でも安全に無視できるよう、
        # サービス未提供時はログ警告のみでホーミング自体は継続する。
        if not client.service_is_ready():
            self.get_logger().warning(f'homing_node: {label} service not available, skipped')
            return

        def _done(fut):
            try:
                res = fut.result()
                if not res.success:
                    self.get_logger().warning(f'homing_node: {label} failed: {res.message}')
            except Exception as exc:
                self.get_logger().error(f'homing_node: {label} call error: {exc}')

        client.call_async(Trigger.Request()).add_done_callback(_done)

    # ---------------- センサ購読 ----------------

    def _on_can_host(self, msg: Int32MultiArray):
        self.can_host_data_ = msg.data

    def _current_motor_joints(self):
        enc1_count = self.can_host_data_[self.enc1_slot_]
        enc2_count = self.can_host_data_[self.enc2_slot_]
        enc1_rad = (enc1_count / self.motor_cpr_) * 2.0 * 3.141592653589793
        enc2_rad = (enc2_count / self.motor_cpr_) * 2.0 * 3.141592653589793
        if self.enc1_is_motor1_:
            motor1_rad, motor2_rad = enc1_rad, enc2_rad
        else:
            motor1_rad, motor2_rad = enc2_rad, enc1_rad
        motor1_joint = self.motor1_sign_ * motor1_rad * self.pulley_radius_m_
        motor2_joint = self.motor2_sign_ * motor2_rad * self.pulley_radius_m_
        return motor1_joint, motor2_joint

    def _switch_triggered(self, slot):
        return self.can_host_data_[slot] == self.switch_triggered_value_

    # ---------------- 制御ループ ----------------

    def _send_velocity(self, motor1_rpm, motor2_rpm):
        data = [0] * SLOT_COUNT
        data[self.robomas_motor1_slot_] = int(motor1_rpm)
        data[self.robomas_motor2_slot_] = int(motor2_rpm)
        msg = Int16MultiArray()
        msg.data = data
        self.tx_pub_.publish(msg)

    def _on_tick(self):
        if self.state_ not in (STATE_HOMING_Z, STATE_HOMING_R):
            return
        if self.can_host_data_ is None:
            return

        elapsed = time.monotonic() - self.phase_start_time_
        if elapsed > self.homing_timeout_sec_:
            self._send_velocity(0.0, 0.0)
            self.state_ = STATE_FAILED
            self._call_trigger_async(self._resume_robomas_cli_, 'resume_robomas_output')
            self.get_logger().error(
                f'homing_node: TIMEOUT during {self.state_} (>{self.homing_timeout_sec_}s). '
                f'motors stopped.')
            return

        if self.state_ == STATE_HOMING_Z:
            if self._switch_triggered(self.z_sw_slot_):
                self._send_velocity(0.0, 0.0)
                m1, m2 = self._current_motor_joints()
                z_raw = self.mix_k_ * (m1 + m2)
                self.z_offset_m_ = self.z_ref_value_m_ - z_raw
                self.get_logger().info(
                    f'homing_node: z-axis limit reached (z_raw={z_raw:.5f}, '
                    f'z_offset_m={self.z_offset_m_:.5f}). start phase=r')
                self.state_ = STATE_HOMING_R
                self.phase_start_time_ = time.monotonic()
                return
            self._send_velocity(
                self.z_home_m1_sign_ * self.homing_velocity_rpm_,
                self.z_home_m2_sign_ * self.homing_velocity_rpm_)

        elif self.state_ == STATE_HOMING_R:
            if self._switch_triggered(self.r_sw_slot_):
                self._send_velocity(0.0, 0.0)
                m1, m2 = self._current_motor_joints()
                r_raw = self.mix_k_ * (m1 - m2)
                r_offset_m = self.r_ref_value_m_ - r_raw
                self.get_logger().info(
                    f'homing_node: r-axis limit reached (r_raw={r_raw:.5f}, '
                    f'r_offset_m={r_offset_m:.5f}). applying offsets to bridge node')
                self._apply_offsets(self.z_offset_m_, r_offset_m)
                self.state_ = STATE_DONE
                self._call_trigger_async(self._resume_robomas_cli_, 'resume_robomas_output')
                return
            self._send_velocity(
                self.r_home_m1_sign_ * self.homing_velocity_rpm_,
                self.r_home_m2_sign_ * self.homing_velocity_rpm_)

    def _apply_offsets(self, z_offset_m, r_offset_m):
        # real_joint_bridge_node(sim表示用)とtrajectory_follower_node(実機MIT指令用)の
        # 両方が独自にz_offset_m/r_offset_mを保持しているため、両方へ反映する
        # (2026-08-29追加。trajectory_follower_node側はrobomas_z/r_offset_mという
        # パラメータ名で、実機出力無効(robomas_device_id未設定)なら値自体は
        # 使われないだけなので、未起動でも安全に無視できる)。
        self._apply_offsets_to(
            self._set_params_cli, 'z_offset_m', 'r_offset_m', z_offset_m, r_offset_m,
            'real_joint_bridge_node')
        self._apply_offsets_to(
            self._set_params_cli_traj_, 'robomas_z_offset_m', 'robomas_r_offset_m',
            z_offset_m, r_offset_m, 'trajectory_follower_node')

    def _apply_offsets_to(self, client, z_param_name, r_param_name,
                           z_offset_m, r_offset_m, label):
        if not client.service_is_ready():
            self.get_logger().error(
                f'homing_node: {label} set_parameters service not available, '
                f'offsets NOT applied. Is {label} running?')
            return
        params = [
            Parameter(name=z_param_name,
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                            double_value=float(z_offset_m))),
            Parameter(name=r_param_name,
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                            double_value=float(r_offset_m))),
        ]
        future = client.call_async(SetParameters.Request(parameters=params))

        def _done(fut):
            try:
                fut.result()
                self.get_logger().info(f'homing_node: offsets applied to {label}')
            except Exception as exc:
                self.get_logger().error(f'homing_node: failed to apply offsets to {label}: {exc}')

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
