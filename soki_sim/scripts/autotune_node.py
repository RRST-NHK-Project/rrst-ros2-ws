#!/usr/bin/env python3
"""z_joint/r_joint(ROBOMAS M2006+C610、差動ミックス)のMIT位置ゲイン
(trajectory_follower_nodeのrobomas_kp/robomas_kd)をz軸・r軸それぞれ別の
試験手順で自動探索するノード。

## なぜ「Z軸ゲイン」「R軸ゲイン」ではなく共通のrobomas_kp/kdを探索するのか
実機の位置制御はモータ単位(M1/M2)のMIT位置ゲインで、joint空間はz=mix_k*(m1+m2)、
r=mix_k*(m1-m2)という差動合成(trajectory_follower_node.py _publish_robomas_commands
参照)。M1とM2に別々のkp/kdを与えても、数式上z方向とr方向のゲインは独立にはならず
必ず相互に漏れ込む(例: r軸だけ動かしてもz側にわずかにドリフトする)。そのため
本ノードはz軸用の試験・r軸用の試験を別々に行えるようにしつつ、最終的には両方に
効く共通の1組のkp/kdを提案する(2026-09-09、ユーザー確認済みの方針)。真に独立した
z/rゲインが必要になった場合は、制御則自体をz/r空間で誤差計算する方式(Python側で
kp_z/kd_z, kp_r/kd_rから力を計算しM1/M2電流へ合成)に変更する必要があり、本ノードの
延長では対応できない。

## 探索手法: Twiddle法(座標降下)
モデル同定が不要でリミットスイッチ付きの実機でも比較的安全に使える手法として、
Twiddle法(各パラメータをkp->kdの順に+d/-d試し、良くなれば採用して探索幅dを
拡大、悪化すれば探索幅を縮小する座標降下法)を採用した。限界感度法(relay-feedback)
は意図的に発振させる必要がありリミット付き機構では危険なため不採用。

評価は、現在位置(baseline)を中心にamplitude_m前後へ2ステップ動かし(step_up/
step_down)、その追従誤差の時間積分(IAE)の合計をスコアとする(小さいほど良い)。
オーバーシュート・振動・追従遅れのいずれもこの1指標に反映される。

## 安全設計
- 開始前提: mixed_joint_states受信済み(=real_joint_bridge_nodeが実機帰還を
  出せている)・estop/リミットスイッチ非作動、のみ。絶対位置の上下限(ストローク長)は
  本ノードでは管理せず、「現在位置からamplitude_mだけ動かす」小振幅試験に留める
  ことと、trajectory_follower_node側の上下限リミットスイッチ安全停止
  (limit_stop_active)に委ねる。従って、操作者はリミットから十分離れた位置に
  機体を置いてから開始すること。
- 実行中、estop_active/limit_stop_activeのいずれかがTrueになった場合、および
  追従誤差がmax_error_m(=amplitude_m*max_error_amplitude_mult)を
  max_error_hold_sec以上超え続けた場合(暴走の疑い)は、即座に試験を中断し、
  開始時に読み取った元のkp/kdへ復元する。
- 探索完了時も、実機には見つかった提案ゲインを反映したままにはせず、必ず元の
  ゲインへ復元する。提案値はautotune_progressへログするのみで、実際に反映するかは
  command_gui_nodeの「ゲイン」タブから操作者が内容を確認した上で明示的に行う
  (既存の「読込は表示のみ、適用は確認ダイアログ付き」という設計方針に合わせる)。
"""

import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

STATE_IDLE = 'idle'
STATE_RUNNING_Z = 'running_z'
STATE_RUNNING_R = 'running_r'
STATE_DONE = 'done'
STATE_FAILED = 'failed'

JOINT_NAMES = ('root_theta_joint', 'z_joint', 'r_joint')


class AutotuneNode(Node):

    def __init__(self):
        super().__init__('autotune_node')

        self.declare_parameter('traj_node_name', 'trajectory_follower_node')
        self.declare_parameter('amplitude_m', 0.02)
        self.declare_parameter('settle_time_sec', 2.0)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('max_iterations', 20)
        self.declare_parameter('kp_step_frac', 0.2)
        self.declare_parameter('kd_step_frac', 0.3)
        self.declare_parameter('kp_step_min', 0.001)
        self.declare_parameter('kd_step_min', 0.0001)
        self.declare_parameter('max_error_amplitude_mult', 2.5)
        self.declare_parameter('max_error_hold_sec', 0.3)

        gp = self.get_parameter
        self._traj_node_name = gp('traj_node_name').value
        self._amplitude_m = float(gp('amplitude_m').value)
        self._settle_time_sec = float(gp('settle_time_sec').value)
        self._sample_period_sec = float(gp('sample_period_sec').value)
        self._max_iterations = int(gp('max_iterations').value)
        self._kp_step_frac = float(gp('kp_step_frac').value)
        self._kd_step_frac = float(gp('kd_step_frac').value)
        self._kp_step_min = float(gp('kp_step_min').value)
        self._kd_step_min = float(gp('kd_step_min').value)
        self._max_error_amplitude_mult = float(gp('max_error_amplitude_mult').value)
        self._max_error_hold_sec = float(gp('max_error_hold_sec').value)

        self._current_positions = {name: None for name in JOINT_NAMES}
        self._estop_active = False
        self._limit_stop_active = False

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, 'mixed_joint_states', self._on_mixed_joint_state, 10)
        self.create_subscription(Bool, 'estop_active', self._on_estop_active, latched_qos)
        self.create_subscription(Bool, 'limit_stop_active', self._on_limit_stop_active, latched_qos)

        self.target_pub_ = self.create_publisher(JointState, 'joint_targets', 10)
        self.state_pub_ = self.create_publisher(String, 'autotune_state', latched_qos)
        self.progress_pub_ = self.create_publisher(String, 'autotune_progress', 10)

        self._get_params_cli = self.create_client(
            GetParameters, f'/{self._traj_node_name}/get_parameters')
        self._set_params_cli = self.create_client(
            SetParameters, f'/{self._traj_node_name}/set_parameters')

        self.create_service(Trigger, 'start_autotune_z', self._on_start_autotune_z)
        self.create_service(Trigger, 'start_autotune_r', self._on_start_autotune_r)
        self.create_service(Trigger, 'stop_autotune', self._on_stop_autotune)

        self.state_ = STATE_IDLE
        self._axis = None            # 'z'/'r'(実行中の軸。停止中はNone)
        self._other_axis = None
        self._baseline = None        # 開始時の{joint_name: 位置}(他軸を固定するため保持)
        self._original_gains = None  # 開始時のkp/kd(終了時に必ず復元する)
        self._request_generation = 0  # stop_autotune等で古い非同期応答を無視するための世代カウンタ
        self._trial_phase = None      # None/'step_up'/'step_down'/'return'
        self._phase_start_time = None
        self._phase_error_integral = 0.0
        self._error_over_since = None
        self._trial_no = 0
        self._params = None
        self._dparams = None
        self._best_score = None
        self._coord_idx = 0
        self._twiddle_phase = None
        self._coord_original = None
        self._iteration = 0

        self._publish_state()
        self.create_timer(self._sample_period_sec, self._on_tick)

        self.get_logger().info(
            "autotune_node started (idle). call 'start_autotune_z'/'start_autotune_r' service to begin.")

    # ---------------- 購読 ----------------

    def _on_mixed_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_positions:
                self._current_positions[name] = pos

    def _on_estop_active(self, msg):
        self._estop_active = msg.data

    def _on_limit_stop_active(self, msg):
        self._limit_stop_active = msg.data

    def _publish_state(self):
        msg = String()
        msg.data = self.state_
        self.state_pub_.publish(msg)

    def _publish_progress(self, text):
        msg = String()
        msg.data = text
        self.progress_pub_.publish(msg)
        self.get_logger().info(f'autotune_node: {text}')

    # ---------------- サービス ----------------

    def _on_start_autotune_z(self, request, response):
        return self._start_axis('z', response)

    def _on_start_autotune_r(self, request, response):
        return self._start_axis('r', response)

    def _start_axis(self, axis, response):
        if self.state_ in (STATE_RUNNING_Z, STATE_RUNNING_R):
            response.success = False
            response.message = f'already running (state={self.state_})'
            return response
        if self._estop_active or self._limit_stop_active:
            response.success = False
            response.message = '緊急停止またはリミットスイッチ作動中は開始できません'
            return response
        if any(v is None for v in self._current_positions.values()):
            response.success = False
            response.message = 'mixed_joint_states未受信です(real_joint_bridge_node起動確認)'
            return response
        if not self._get_params_cli.service_is_ready() or not self._set_params_cli.service_is_ready():
            response.success = False
            response.message = f'{self._traj_node_name}のparameterサービスに接続できません(未起動?)'
            return response

        self._axis = axis
        self._other_axis = 'r' if axis == 'z' else 'z'
        self._baseline = dict(self._current_positions)
        self._request_generation += 1
        self.state_ = STATE_RUNNING_Z if axis == 'z' else STATE_RUNNING_R
        self._publish_state()
        self._publish_progress(
            f'{axis}軸: 開始 (baseline={self._baseline[f"{axis}_joint"]:.5f}m, '
            f'amplitude={self._amplitude_m}m)。前後に往復させながらkp/kdを探索します。')

        self._fetch_original_gains()
        response.success = True
        response.message = f'{axis}軸の自動ゲイン調整を開始しました'
        return response

    def _on_stop_autotune(self, request, response):
        if self.state_ not in (STATE_RUNNING_Z, STATE_RUNNING_R):
            response.success = False
            response.message = '実行中ではありません'
            return response
        self._abort('ユーザーによる中断', restore=True, final_state=STATE_IDLE)
        response.success = True
        response.message = '中断し、元のゲインへ復元しました'
        return response

    # ---------------- ゲイン取得/設定/復元 ----------------

    def _fetch_original_gains(self):
        request_id = self._request_generation
        future = self._get_params_cli.call_async(
            GetParameters.Request(names=['robomas_kp', 'robomas_kd']))

        def _done(fut):
            if request_id != self._request_generation:
                return
            try:
                values = fut.result().values
                kp, kd = values[0].double_value, values[1].double_value
            except Exception as exc:
                self._abort(f'現在ゲイン取得失敗: {exc}', restore=False, final_state=STATE_FAILED)
                return
            self._original_gains = {'robomas_kp': kp, 'robomas_kd': kd}
            self._begin_twiddle(kp, kd)

        future.add_done_callback(_done)

    def _set_gains(self, kp, kd, on_done):
        future = self._set_params_cli.call_async(SetParameters.Request(parameters=[
            Parameter(name='robomas_kp',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp))),
            Parameter(name='robomas_kd',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd))),
        ]))
        request_id = self._request_generation

        def _done(fut):
            if request_id != self._request_generation:
                return
            try:
                fut.result()
            except Exception as exc:
                self._abort(f'ゲイン設定失敗: {exc}', restore=False, final_state=STATE_FAILED)
                return
            on_done()

        future.add_done_callback(_done)

    def _restore_original_gains_unconditionally(self):
        """abort時専用。世代カウンタは既に進めた後なので通常の_set_gainsの
        コールバック無効化ロジックは通さず、応答を待たず投げっぱなしで送る
        (安全復元を確実に発行することを優先する)。"""
        if self._original_gains is None:
            return
        kp = self._original_gains['robomas_kp']
        kd = self._original_gains['robomas_kd']
        self._set_params_cli.call_async(SetParameters.Request(parameters=[
            Parameter(name='robomas_kp',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp))),
            Parameter(name='robomas_kd',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd))),
        ]))

    # ---------------- Twiddle(座標降下)によるkp/kd探索 ----------------

    def _begin_twiddle(self, kp0, kd0):
        self._params = [kp0, kd0]
        self._dparams = [
            max(kp0 * self._kp_step_frac, self._kp_step_min),
            max(kd0 * self._kd_step_frac, self._kd_step_min),
        ]
        self._best_score = None
        self._coord_idx = 0
        self._iteration = 0
        self._trial_no = 0
        self._run_trial(list(self._params))

    def _run_trial(self, candidate):
        kp, kd = candidate
        self._trial_no += 1
        self._phase_error_integral = 0.0
        self._error_over_since = None
        self._publish_progress(
            f'{self._axis}軸 試行{self._trial_no}: kp={kp:.6f} kd={kd:.6f} を試験中...')

        def _after_set():
            self._trial_phase = 'step_up'
            self._phase_start_time = time.monotonic()
            self._send_target_for_phase('step_up')

        self._set_gains(kp, kd, _after_set)

    def _send_target_for_phase(self, phase):
        axis_joint = f'{self._axis}_joint'
        other_joint = f'{self._other_axis}_joint'
        if phase == 'step_up':
            axis_val = self._baseline[axis_joint] + self._amplitude_m
        elif phase == 'step_down':
            axis_val = self._baseline[axis_joint] - self._amplitude_m
        else:
            axis_val = self._baseline[axis_joint]

        theta = self._baseline['root_theta_joint']
        other_val = self._baseline[other_joint]
        z = axis_val if self._axis == 'z' else other_val
        r = axis_val if self._axis == 'r' else other_val

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [theta, z, r]
        self.target_pub_.publish(msg)

    def _on_trial_scored(self, score):
        best_str = f' (best={self._best_score:.5f})' if self._best_score is not None else ''
        self._publish_progress(
            f'{self._axis}軸 試行{self._trial_no} 完了: '
            f'kp={self._params[0]:.6f} kd={self._params[1]:.6f} score={score:.5f}{best_str}')

        if self._best_score is None:
            # 初回(パラメータ変更前)の基準スコア
            self._best_score = score
            self._start_coord_trial()
            return

        i = self._coord_idx
        if self._twiddle_phase == 'try_plus':
            if score < self._best_score:
                self._best_score = score
                self._dparams[i] *= 1.1
                self._finish_coord()
            else:
                self._params[i] = self._coord_original - self._dparams[i]
                self._twiddle_phase = 'try_minus'
                self._run_trial(list(self._params))
        elif self._twiddle_phase == 'try_minus':
            if score < self._best_score:
                self._best_score = score
                self._dparams[i] *= 1.1
            else:
                self._params[i] = self._coord_original
                self._dparams[i] *= 0.9
            self._finish_coord()

    def _start_coord_trial(self):
        i = self._coord_idx
        self._coord_original = self._params[i]
        self._params[i] = self._coord_original + self._dparams[i]
        self._twiddle_phase = 'try_plus'
        self._run_trial(list(self._params))

    def _finish_coord(self):
        self._coord_idx = (self._coord_idx + 1) % 2
        if self._coord_idx == 0:
            self._iteration += 1
        converged = self._dparams[0] < self._kp_step_min and self._dparams[1] < self._kd_step_min
        if self._iteration >= self._max_iterations or converged:
            self._finish_autotune()
        else:
            self._start_coord_trial()

    def _finish_autotune(self):
        axis = self._axis
        kp, kd = self._params
        self._publish_progress(
            f'{axis}軸: 完了。提案ゲイン kp={kp:.6f} kd={kd:.6f} (score={self._best_score:.5f})。'
            f'実機ゲインは元の値へ復元済みです。内容を確認の上、GUIの「ゲイン」タブから'
            f'手動で入力・適用してください。')
        self._trial_phase = None
        self._set_gains(
            self._original_gains['robomas_kp'], self._original_gains['robomas_kd'],
            lambda: self._enter_terminal_state(STATE_DONE))

    def _enter_terminal_state(self, final_state):
        self.state_ = final_state
        self._publish_state()
        self._axis = None
        self._other_axis = None

    def _abort(self, reason, restore, final_state):
        axis = self._axis
        self._request_generation += 1  # 進行中の非同期コールバック(GetParameters/SetParameters)を無効化
        self._trial_phase = None
        if axis is not None:
            self._send_target_for_phase('return')  # baseline位置へ戻す指令を送っておく
        self._publish_progress(f'{axis}軸: 中断/失敗 ({reason})')
        if restore:
            self._restore_original_gains_unconditionally()
        self._enter_terminal_state(final_state)

    # ---------------- 制御ループ ----------------

    def _on_tick(self):
        if self.state_ not in (STATE_RUNNING_Z, STATE_RUNNING_R):
            return
        if self._estop_active or self._limit_stop_active:
            self._abort('estopまたはリミットスイッチ作動を検出', restore=True, final_state=STATE_FAILED)
            return
        if self._trial_phase not in ('step_up', 'step_down', 'return'):
            return  # ゲイン設定/パラメータ取得の非同期応答待ち

        axis_joint = f'{self._axis}_joint'
        actual = self._current_positions[axis_joint]

        if self._trial_phase in ('step_up', 'step_down'):
            target = self._baseline[axis_joint] + (
                self._amplitude_m if self._trial_phase == 'step_up' else -self._amplitude_m)
            error = abs(target - actual)
            self._phase_error_integral += error * self._sample_period_sec

            max_error_m = self._amplitude_m * self._max_error_amplitude_mult
            if error > max_error_m:
                if self._error_over_since is None:
                    self._error_over_since = time.monotonic()
                elif time.monotonic() - self._error_over_since > self._max_error_hold_sec:
                    self._abort(
                        f'{self._axis}軸の追従誤差が{max_error_m:.4f}mを超えて継続(暴走の疑い)',
                        restore=True, final_state=STATE_FAILED)
                    return
            else:
                self._error_over_since = None

        if time.monotonic() - self._phase_start_time < self._settle_time_sec:
            return

        if self._trial_phase == 'step_up':
            self._trial_phase = 'step_down'
            self._phase_start_time = time.monotonic()
            self._send_target_for_phase('step_down')
        elif self._trial_phase == 'step_down':
            self._trial_phase = 'return'
            self._phase_start_time = time.monotonic()
            self._send_target_for_phase('return')
        elif self._trial_phase == 'return':
            score = self._phase_error_integral
            self._trial_phase = None
            self._on_trial_scored(score)


def main(args=None):
    rclpy.init(args=args)
    node = AutotuneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
