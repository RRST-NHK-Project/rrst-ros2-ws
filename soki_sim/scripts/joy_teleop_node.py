#!/usr/bin/env python3
"""
soki_sim: joyパッケージのjoy_node(/joy, sensor_msgs/Joy)を購読し、スティック/十字キー
入力を関節空間で直接ジョグしてtrajectory_follower_nodeの/joint_targetsへpublishする
手動操作ノード。

操作割り当て(デフォルト。実際のコントローラのaxes/buttons番号は`ros2 topic echo /joy`で
確認し、axis_theta/axis_z/axis_r/axis_tip_theta・invert_*・pump_toggle_buttonパラメータで
合わせること):
  左スティック左右 -> root_theta_joint (axis_theta, デフォルト0)
  右スティック上下 -> z_joint          (axis_z,     デフォルト4)
  右スティック左右 -> tip_theta_joint  (axis_tip_theta, デフォルト3。2026-09-03
                                         追加。手先θ、continuous(可動域制限なし)。
                                         zと同じ右スティックに割り当て)
  十字キー上下     -> r_joint          (axis_r,     デフォルト7。多くのLinux
                                         ジョイスティックドライバでは十字キーが
                                         axes配列の末尾2要素(X,Y)として出てくる)
  ○(丸)ボタン     -> ポンプON/OFFトグル (pump_toggle_button, デフォルト1。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加。hand_node(/hand_pump_on・
                                         /hand_pump_off)へTriggerサービスを呼ぶ。
                                         現在のON/OFF状態はhand_nodeがpublishする
                                         hand_pump_state(std_msgs/Bool)を購読して
                                         判定する(GUIハンドパネルからの操作と
                                         状態がズレないようにするため、本ノード
                                         側ではローカルに推測しない))
  ×(バツ)ボタン   -> 回収実行の指示    (pickup_confirm_button, デフォルト0。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加。command_gui_nodeの回収
                                         シーケンスが「ワーク手前で自動停止→回収
                                         実行待ち」の状態のとき、command_gui_nodeの
                                         /pick_sequence_confirmサービスを呼んで
                                         続行させる(GUIの「回収実行」ボタンと同じ
                                         効果))

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
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

ARM_LENGTH = 1.244
Z_LOWER, Z_UPPER = 0.0, 0.432
R_LOWER, R_UPPER = -ARM_LENGTH / 2.0, ARM_LENGTH / 2.0

# root_theta: 実機CubeMars(MITモード)の指令可能範囲(±12.5rad、アクチュエータ軸)を
# 外部減速比(112/24)で関節角度に変換した値。command_gui_node.pyのROOT_THETA_*、
# soki_sim.urdf.xacroのroot_theta_limitと一致させること。note/hardware_mapping.txt
# 参照(2026-08-27、z/rは元々クランプされていたがthetaだけ無制限だったため追加)。
ROOT_THETA_REDUCTION = 112.0 / 24.0
ROOT_THETA_LIMIT = 12.5 / ROOT_THETA_REDUCTION
ROOT_THETA_LOWER, ROOT_THETA_UPPER = -ROOT_THETA_LIMIT, ROOT_THETA_LIMIT


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
        # 手先θ(tip_theta_joint)の手動ジョグ軸(2026-09-03追加)。右スティック
        # 左右を想定(z=右スティック上下=axis4と対にした既定値)、-1で無効。
        self.declare_parameter('axis_tip_theta', 3)
        self.declare_parameter('invert_theta', False)
        self.declare_parameter('invert_z', False)
        self.declare_parameter('invert_r', False)
        self.declare_parameter('invert_tip_theta', False)
        # -1ならデッドマンボタン無効(常時有効)。実機ではボタンを割り当てて
        # 誤操作による意図しない動作を防ぐことを推奨。
        self.declare_parameter('enable_button', -1)
        self.declare_parameter('theta_speed', 1.0)   # rad/s (フル入力時)
        self.declare_parameter('z_speed', 0.2)       # m/s (フル入力時)
        self.declare_parameter('r_speed', 0.2)       # m/s (フル入力時)
        self.declare_parameter('tip_theta_speed', 1.0)  # rad/s (フル入力時、2026-09-03追加)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('update_rate_hz', 50.0)
        # ポンプON/OFFトグル用ボタン(2026-09-03追加)。-1ならボタン操作無効。
        self.declare_parameter('pump_toggle_button', 1)
        # 回収実行の指示ボタン(2026-09-03追加)。-1ならボタン操作無効。
        self.declare_parameter('pickup_confirm_button', 0)

        self.axis_theta_ = int(self.get_parameter('axis_theta').value)
        self.axis_z_ = int(self.get_parameter('axis_z').value)
        self.axis_r_ = int(self.get_parameter('axis_r').value)
        self.axis_tip_theta_ = int(self.get_parameter('axis_tip_theta').value)
        self.sign_theta_ = -1.0 if self.get_parameter('invert_theta').value else 1.0
        self.sign_z_ = -1.0 if self.get_parameter('invert_z').value else 1.0
        self.sign_r_ = -1.0 if self.get_parameter('invert_r').value else 1.0
        self.sign_tip_theta_ = -1.0 if self.get_parameter('invert_tip_theta').value else 1.0
        self.enable_button_ = int(self.get_parameter('enable_button').value)
        self.theta_speed_ = float(self.get_parameter('theta_speed').value)
        self.z_speed_ = float(self.get_parameter('z_speed').value)
        self.r_speed_ = float(self.get_parameter('r_speed').value)
        self.tip_theta_speed_ = float(self.get_parameter('tip_theta_speed').value)
        self.deadzone_ = float(self.get_parameter('deadzone').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.dt_ = 1.0 / update_rate_hz
        self.pump_toggle_button_ = int(self.get_parameter('pump_toggle_button').value)
        self.pickup_confirm_button_ = int(self.get_parameter('pickup_confirm_button').value)

        # 現在の目標関節角度(スティック/十字キー入力をここへ積分していく)。
        # /mixed_joint_statesを受信するまでは、trajectory_follower_node起動直後の
        # 実際の静止姿勢(全関節0.0)に合わせておく。ここを可動域の中央など実際と
        # 異なる値にすると、/mixed_joint_states受信前に一部の軸だけ操作した場合でも
        # 同じJointStateメッセージに含まれる他の軸がその適当な初期値へ動いてしまう。
        self.target_theta_ = 0.0
        self.target_z_ = 0.0
        self.target_r_ = 0.0
        self.target_tip_theta_ = 0.0
        self.has_current_state_ = False
        self.has_tip_theta_state_ = False

        self.latest_joy_ = None
        self.pub_ = self.create_publisher(JointState, 'joint_targets', 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.create_subscription(JointState, 'mixed_joint_states', self._on_mixed_joint_state, 10)
        self.timer_ = self.create_timer(self.dt_, self._timer_callback)

        # ポンプON/OFFトグル(○ボタン、2026-09-03追加)。現在のON/OFF状態は
        # hand_node起動時とON/OFF操作の度にpublishされるhand_pump_stateを購読して
        # 判定する(GUIハンドパネル経由の操作とも状態がズレないようにするため)。
        self._pump_on_ = False
        self._prev_pump_button_pressed_ = False
        # hand_node側はtransient_local(depth=1)でpublishしているため、本ノード側も
        # 同じdurabilityを要求しないと起動時点の状態を受け取れない
        # (command_gui_node.pyの同トピック購読部のコメント参照)。
        pump_state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'hand_pump_state', self._on_pump_state, pump_state_qos)
        self._pump_on_client_ = self.create_client(Trigger, 'hand_pump_on')
        self._pump_off_client_ = self.create_client(Trigger, 'hand_pump_off')

        # 回収実行の指示ボタン(×ボタン、2026-09-03追加)。command_gui_node側の
        # 回収シーケンスが「ワーク手前で自動停止→回収実行待ち」のときに続行させる。
        self._prev_pickup_confirm_pressed_ = False
        self._pickup_confirm_client_ = self.create_client(Trigger, 'pick_sequence_confirm')

        # theta_speed/z_speed/r_speedは起動時にself.*_speed_へ取り込んだ後は
        # 参照されないため、command_gui_nodeの「手動操作(joy)速度」パネル等で
        # ros2 param set しても反映されなかった。on_set_parameters_callbackで
        # キャッシュ側も更新することで実行中に反映させる。
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'joy_teleop_node started: axis_theta={self.axis_theta_}, axis_z={self.axis_z_}, '
            f'axis_r={self.axis_r_}, axis_tip_theta={self.axis_tip_theta_}, '
            f'enable_button={self.enable_button_}, '
            f'theta_speed={self.theta_speed_}rad/s, z_speed={self.z_speed_}m/s, '
            f'r_speed={self.r_speed_}m/s, tip_theta_speed={self.tip_theta_speed_}rad/s, '
            f'rate={update_rate_hz}Hz')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name in ('theta_speed', 'z_speed', 'r_speed', 'tip_theta_speed') and p.value <= 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be positive')
        for p in params:
            if p.name == 'theta_speed':
                self.theta_speed_ = float(p.value)
            elif p.name == 'z_speed':
                self.z_speed_ = float(p.value)
            elif p.name == 'tip_theta_speed':
                self.tip_theta_speed_ = float(p.value)
            elif p.name == 'r_speed':
                self.r_speed_ = float(p.value)
        return SetParametersResult(successful=True)

    def _on_joy(self, msg: Joy):
        self.latest_joy_ = msg

    def _on_pump_state(self, msg: Bool):
        self._pump_on_ = msg.data

    def _update_pump_toggle(self, msg: Joy):
        """○ボタンの立ち上がりエッジで、hand_nodeから購読済みの現在のポンプ状態と
        逆のTriggerサービス(hand_pump_on/hand_pump_off)を呼ぶ(2026-09-03追加)。
        移動系のenable_button(デッドマン)とは独立(GUIハンドパネルのポンプON/OFF
        ボタンも確認ダイアログ無しの通常操作扱いのため、同様に扱う)。"""
        if self.pump_toggle_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.pump_toggle_button_ < len(buttons)
                   and bool(buttons[self.pump_toggle_button_]))
        if pressed and not self._prev_pump_button_pressed_:
            client = self._pump_off_client_ if self._pump_on_ else self._pump_on_client_
            if client.service_is_ready():
                client.call_async(Trigger.Request())
            else:
                self.get_logger().warning(
                    'joy_teleop_node: hand_nodeのポンプサービスに接続できません(未起動?)')
        self._prev_pump_button_pressed_ = pressed

    def _update_pickup_confirm(self, msg: Joy):
        """×ボタンの立ち上がりエッジで、command_gui_nodeの/pick_sequence_confirm
        (std_srvs/Trigger)を呼ぶ(2026-09-03追加)。回収シーケンスが「ワーク手前で
        自動停止→回収実行待ち」の状態でなければcommand_gui_node側が失敗を返すだけ
        (安全側)。移動系のenable_button(デッドマン)とは独立に扱う(ポンプ
        トグルボタンと同じ理由)。"""
        if self.pickup_confirm_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.pickup_confirm_button_ < len(buttons)
                   and bool(buttons[self.pickup_confirm_button_]))
        if pressed and not self._prev_pickup_confirm_pressed_:
            if self._pickup_confirm_client_.service_is_ready():
                self._pickup_confirm_client_.call_async(Trigger.Request())
            else:
                self.get_logger().warning(
                    'joy_teleop_node: command_gui_nodeの回収確認サービスに接続できません(GUI未起動?)')
        self._prev_pickup_confirm_pressed_ = pressed

    def _on_mixed_joint_state(self, msg: JointState):
        try:
            self._current_theta_ = msg.position[msg.name.index('root_theta_joint')]
            self._current_z_ = msg.position[msg.name.index('z_joint')]
            self._current_r_ = msg.position[msg.name.index('r_joint')]
        except ValueError:
            return
        self.has_current_state_ = True
        # tip_theta_jointはtrajectory_follower_nodeの起動構成次第で無い場合もある
        # ため(root_theta/z/rと違い)、他の3関節とは別に任意扱いにする
        # (2026-09-03追加)。
        try:
            self._current_tip_theta_ = msg.position[msg.name.index('tip_theta_joint')]
            self.has_tip_theta_state_ = True
        except ValueError:
            pass

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
        self._update_pump_toggle(msg)
        self._update_pickup_confirm(msg)
        enabled = self._is_enabled(msg)

        theta_in = apply_deadzone(self._axis(msg.axes, self.axis_theta_), self.deadzone_) * self.sign_theta_
        z_in = apply_deadzone(self._axis(msg.axes, self.axis_z_), self.deadzone_) * self.sign_z_
        r_in = apply_deadzone(self._axis(msg.axes, self.axis_r_), self.deadzone_) * self.sign_r_
        tip_theta_in = (apply_deadzone(self._axis(msg.axes, self.axis_tip_theta_), self.deadzone_)
                        * self.sign_tip_theta_)

        names = []
        positions = []

        # レート方式。入力が中立/デッドマン未押下の間は目標を現在値に同期する
        # だけでpublishに含めない(離した位置を保持する。autoモードの動作を
        # 妨げず、モード切替時の急変も防ぐ)。
        if enabled and theta_in != 0.0:
            self.target_theta_ = clamp(
                self.target_theta_ + theta_in * self.theta_speed_ * self.dt_,
                ROOT_THETA_LOWER, ROOT_THETA_UPPER)
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

        # tip_theta_jointはcontinuous(可動域制限なし)なのでclampしない。
        # trajectory_follower_nodeがtip_theta_joint未構成の場合はpublishしても
        # target_callback側で無視されるだけなので、has_tip_theta_state_の有無に
        # 関わらず常に試みる(2026-09-03追加)。
        if enabled and tip_theta_in != 0.0:
            self.target_tip_theta_ += tip_theta_in * self.tip_theta_speed_ * self.dt_
            names.append('tip_theta_joint')
            positions.append(self.target_tip_theta_)
        elif self.has_tip_theta_state_:
            self.target_tip_theta_ = self._current_tip_theta_

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
