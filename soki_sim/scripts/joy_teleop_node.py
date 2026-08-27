#!/usr/bin/env python3
"""
soki_sim: joyパッケージのjoy_node(/joy, sensor_msgs/Joy)を購読し、スティック/十字キー
入力を関節空間で直接ジョグしてtrajectory_follower_nodeの/joint_targetsへpublishする
手動操作ノード。

操作割り当て(デフォルト。実際のコントローラのaxes番号は`ros2 topic echo /joy`で
確認し、axis_theta/axis_z/axis_r・invert_*パラメータで合わせること):
  左スティック左右 -> root_theta_joint (axis_theta, デフォルト0)
  右スティック上下 -> z_joint          (axis_z,     デフォルト4)
  十字キー上下     -> r_joint          (axis_r,     デフォルト7。多くのLinux
                                         ジョイスティックドライバでは十字キーが
                                         axes配列の末尾2要素(X,Y)として出てくる)

いずれもレート方式: 倒している間、target += 入力値*speed*dt で積分し続ける。
入力が中立/デッドマン未押下の間は目標を/mixed_joint_statesの現在値に同期する
だけでpublishに含めない(離した位置を保持する。autoモードの動作を妨げず、
モード切替時の急変も防ぐ)。
(一度スティックの倒し具合をそのままZ_LOWER〜Z_UPPERへ線形マッピングする絶対位置
方式をzに試したが、スティックが物理的に中立へ戻るたびにzも可動域中央へ戻って
しまう("Zが勝手に戻る")ため、他の軸と同じレート方式に戻した)

/joint_targetsにはheader.frame_id='manual'を付けてpublishする(command_gui_nodeは
'auto')。trajectory_follower_node側のcontrol_modeパラメータ('auto'/'manual'/'both')
がこのタグを見て受け付けるかどうかを判定するため、本ノード自身はモードを意識せず
常にpublishしてよい(command_gui_nodeの「動作モード」パネルで一元的に切り替える)。
1つのJointStateメッセージには、その周期で実際に動かす関節だけを含める。

体感の遅延はほぼtrajectory_follower_node側のmax_velocity/max_acceleration
(GUIのクリック移動用に控えめな値になっている)に起因する。目標側のspeedパラメータ
がこれより速いと、目標だけ先に進んで実際の追従が遅れて感じる(thetaが良好で
r/zが遅く感じたのはこのミスマッチが原因。launch/display.launch.py側でz/rの
max_velocity/max_accelerationを引き上げ済み)。本ノード自体は追加の遅延を避ける
ためupdate_rate_hzのデフォルトを50Hzにしている。
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

ARM_LENGTH = 1.244
Z_LOWER, Z_UPPER = 0.0, 0.432
R_LOWER, R_UPPER = -ARM_LENGTH / 2.0, ARM_LENGTH / 2.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        self.declare_parameter('axis_theta', 0)
        self.declare_parameter('axis_z', 4)
        self.declare_parameter('axis_r', 7)
        self.declare_parameter('invert_theta', False)
        self.declare_parameter('invert_z', False)
        self.declare_parameter('invert_r', False)
        # -1ならデッドマンボタン無効(常時有効)。実機ではボタンを割り当てて
        # 誤操作による意図しない動作を防ぐことを推奨。
        self.declare_parameter('enable_button', -1)
        self.declare_parameter('theta_speed', 1.0)   # rad/s (フル入力時)
        self.declare_parameter('z_speed', 0.2)       # m/s (フル入力時)
        self.declare_parameter('r_speed', 0.2)       # m/s (フル入力時)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('update_rate_hz', 50.0)

        self.axis_theta_ = int(self.get_parameter('axis_theta').value)
        self.axis_z_ = int(self.get_parameter('axis_z').value)
        self.axis_r_ = int(self.get_parameter('axis_r').value)
        self.sign_theta_ = -1.0 if self.get_parameter('invert_theta').value else 1.0
        self.sign_z_ = -1.0 if self.get_parameter('invert_z').value else 1.0
        self.sign_r_ = -1.0 if self.get_parameter('invert_r').value else 1.0
        self.enable_button_ = int(self.get_parameter('enable_button').value)
        self.theta_speed_ = float(self.get_parameter('theta_speed').value)
        self.z_speed_ = float(self.get_parameter('z_speed').value)
        self.r_speed_ = float(self.get_parameter('r_speed').value)
        self.deadzone_ = float(self.get_parameter('deadzone').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.dt_ = 1.0 / update_rate_hz

        # 現在の目標関節角度(スティック/十字キー入力をここへ積分していく)。
        # /mixed_joint_statesを受信するまでは、trajectory_follower_node起動直後の
        # 実際の静止姿勢(全関節0.0)に合わせておく。ここを可動域の中央など実際と
        # 異なる値にすると、/mixed_joint_states受信前に一部の軸だけ操作した場合でも
        # 同じJointStateメッセージに含まれる他の軸がその適当な初期値へ動いてしまう。
        self.target_theta_ = 0.0
        self.target_z_ = 0.0
        self.target_r_ = 0.0
        self.has_current_state_ = False

        self.latest_joy_ = None
        self.pub_ = self.create_publisher(JointState, 'joint_targets', 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_subscription(JointState, 'mixed_joint_states', self._on_mixed_joint_state, 10)
        self.timer_ = self.create_timer(self.dt_, self._timer_callback)

        # theta_speed/z_speed/r_speedは起動時にself.*_speed_へ取り込んだ後は
        # 参照されないため、command_gui_nodeの「手動操作(joy)速度」パネル等で
        # ros2 param set しても反映されなかった。on_set_parameters_callbackで
        # キャッシュ側も更新することで実行中に反映させる。
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'joy_teleop_node started: axis_theta={self.axis_theta_}, axis_z={self.axis_z_}, '
            f'axis_r={self.axis_r_}, enable_button={self.enable_button_}, '
            f'theta_speed={self.theta_speed_}rad/s, z_speed={self.z_speed_}m/s, '
            f'r_speed={self.r_speed_}m/s, rate={update_rate_hz}Hz')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name in ('theta_speed', 'z_speed', 'r_speed') and p.value <= 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be positive')
        for p in params:
            if p.name == 'theta_speed':
                self.theta_speed_ = float(p.value)
            elif p.name == 'z_speed':
                self.z_speed_ = float(p.value)
            elif p.name == 'r_speed':
                self.r_speed_ = float(p.value)
        return SetParametersResult(successful=True)

    def _on_joy(self, msg: Joy):
        self.latest_joy_ = msg

    def _on_mixed_joint_state(self, msg: JointState):
        try:
            self._current_theta_ = msg.position[msg.name.index('root_theta_joint')]
            self._current_z_ = msg.position[msg.name.index('z_joint')]
            self._current_r_ = msg.position[msg.name.index('r_joint')]
        except ValueError:
            return
        self.has_current_state_ = True

    def _axis(self, axes, index):
        return axes[index] if 0 <= index < len(axes) else 0.0

    def _is_enabled(self, msg: Joy):
        if self.enable_button_ < 0:
            return True
        buttons = msg.buttons
        return 0 <= self.enable_button_ < len(buttons) and bool(buttons[self.enable_button_])

    def _timer_callback(self):
        msg = self.latest_joy_
        if msg is None:
            return
        enabled = self._is_enabled(msg)

        theta_in = apply_deadzone(self._axis(msg.axes, self.axis_theta_), self.deadzone_) * self.sign_theta_
        z_in = apply_deadzone(self._axis(msg.axes, self.axis_z_), self.deadzone_) * self.sign_z_
        r_in = apply_deadzone(self._axis(msg.axes, self.axis_r_), self.deadzone_) * self.sign_r_

        names = []
        positions = []

        # レート方式。入力が中立/デッドマン未押下の間は目標を現在値に同期する
        # だけでpublishに含めない(離した位置を保持する。autoモードの動作を
        # 妨げず、モード切替時の急変も防ぐ)。
        if enabled and theta_in != 0.0:
            self.target_theta_ += theta_in * self.theta_speed_ * self.dt_
            names.append('root_theta_joint')
            positions.append(self.target_theta_)
        elif self.has_current_state_:
            self.target_theta_ = self._current_theta_

        if enabled and z_in != 0.0:
            self.target_z_ = clamp(self.target_z_ + z_in * self.z_speed_ * self.dt_, Z_LOWER, Z_UPPER)
            names.append('z_joint')
            positions.append(self.target_z_)
        elif self.has_current_state_:
            self.target_z_ = self._current_z_

        if enabled and r_in != 0.0:
            self.target_r_ = clamp(self.target_r_ + r_in * self.r_speed_ * self.dt_, R_LOWER, R_UPPER)
            names.append('r_joint')
            positions.append(self.target_r_)
        elif self.has_current_state_:
            self.target_r_ = self._current_r_

        if not names:
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'manual'
        out.name = names
        out.position = positions
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
