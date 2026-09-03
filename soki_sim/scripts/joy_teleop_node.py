#!/usr/bin/env python3
"""
soki_sim: joyパッケージのjoy_node(/joy, sensor_msgs/Joy)を購読し、スティック/十字キー
入力を関節空間で直接ジョグしてtrajectory_follower_nodeの/joint_targetsへpublishする
手動操作ノード。

操作割り当て(デフォルト。実際のコントローラのaxes/buttons番号は`ros2 topic echo /joy`で
確認し、axis_theta/axis_z/axis_r/axis_tip_theta/axis_x/axis_y・invert_*・
pump_toggle_buttonパラメータで合わせること):
  右スティック上下 -> z_joint (axis_z, デフォルト4。2026-09-03、ユーザー指定:
                                         「Zは右スティック上下で」により変更
                                         (以前は左スティック上下)。XYモード・
                                         関節モードのどちらでも常にこの軸が
                                         z_jointを操作する)
  右スティック左右 -> tip_theta_joint  (axis_tip_theta, デフォルト3。手先θ、
                                         continuous(可動域制限なし))
  左スティック左右 -> X(XYモード時、既定)/root_theta_joint(関節モード時)
                                         (axis_x/axis_theta、いずれもデフォルト0。
                                         下記SHAREボタンの項目参照)
  左スティック上下 -> Y(XYモード時、既定)/r_joint(関節モード時)
                                         (axis_y/axis_r、いずれもデフォルト1。
                                         2026-09-03、ユーザー指定:「XYは左スティック」
                                         によりaxis_yを4→1に変更。axis_rも右
                                         スティック上下(z_joint)との競合を避けるため
                                         4→1に変更(以前はaxis_r=4=axis_zの旧値と
                                         同じ軸で、z_jointが右スティック上下に
                                         移った今そのままだと関節モードでr_jointと
                                         z_jointが同時に動いてしまうため))
  △(三角)ボタン   -> ポンプON/OFFトグル (pump_toggle_button, デフォルト2。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加、同日ボタン再割当。
                                         hand_node(/hand_pump_on・/hand_pump_off)へ
                                         Triggerサービスを呼ぶ。現在のON/OFF状態は
                                         hand_nodeがpublishするhand_pump_state
                                         (std_msgs/Bool)を購読して判定する(GUI
                                         ハンドパネルからの操作と状態がズレない
                                         ようにするため、本ノード側ではローカルに
                                         推測しない))
  ×(バツ)ボタン   -> 選択中ワークへ移動 (pickup_confirm_button, デフォルト0。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加。立ち上がりエッジ即時で
                                         command_gui_nodeの/pick_sequence_move
                                         サービスを呼び、矢印キー(D-pad)で選択中の
                                         ワークへの回収シーケンスを開始する
                                         (GUIの「回収実行」ボタンの「移動」側と
                                         同じ効果。他のシーケンス実行中でも即座に
                                         中断して切り替わる)。2026-09-03、同日
                                         一時的に「×長押し=回収実行確定」も
                                         この同じボタンに割り当てていたが、移動
                                         側が即座にシーケンスを中断・やり直す
                                         仕様になったため、長押しのつもりで押した
                                         瞬間に移動が先に発火して回収実行待ち状態を
                                         壊してしまう不具合が起きた。ユーザー指定:
                                         「回収ボタンをバツ長押しからPSボタンに
                                         変更」により、確定は別ボタン(下記PSボタン)
                                         へ分離し、×は単純な即時押下のみに戻した)
  PSボタン         -> 回収実行(確定)   (pick_confirm_button, デフォルト10。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加。立ち上がりエッジ即時で
                                         command_gui_nodeの/pick_sequence_confirm
                                         サービスを呼び、「ワーク手前で自動停止→
                                         回収実行待ち」の状態を1回だけ進める
                                         (GUIの「回収実行」ボタンの「確定」側と
                                         同じ効果。誤操作でワークに接触・吸着して
                                         しまうことを防ぐため、移動用の×ボタンとは
                                         意図的に別ボタンにしている))
  □(四角)ボタン   -> L4へ移動          (shoot_start_l4_button, デフォルト3。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加、同日ボタン再割当。
                                         command_gui_nodeの/shoot_sequence_start_l4
                                         サービスを呼び、シューティングエリアL4へ、
                                         安全高度を維持したまま向かうだけの投入
                                         シーケンスを実行させる(GUIの「L4へ移動」
                                         ボタンと同じ効果))
  ○(丸)ボタン     -> R4へ移動          (shoot_start_r4_button, デフォルト1。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加、同日ボタン再割当。
                                         command_gui_nodeの/shoot_sequence_start_r4
                                         サービスを呼び、シューティングエリアR4へ
                                         同様に向かわせる(GUIの「R4へ移動」ボタンと
                                         同じ効果))
  OPTIONSボタン    -> 手先θのroot_theta追従トグル (tip_theta_follow_theta_button,
                                         デフォルト9。PS4/PS5コントローラの
                                         一般的なLinuxドライバ割り当てを仮定した値、
                                         実機で要確認。2026-09-03追加、ユーザー指定:
                                         「OPTIONSボタンで手先θを根本θに追従させる
                                         か切り替えられるように」「デフォルトを
                                         自動シーケンスで実行に」。ONの間は
                                         tip_theta_joint = -root_theta_jointを
                                         毎周期指令し続け(回収シーケンスと同じ
                                         追従式)、右スティック左右による手動ジョグは
                                         無視する。OFFにすると従来通り右スティック
                                         左右で独立にジョグできる。既定でON。
                                         command_gui_nodeのset_tip_theta_follow_theta
                                         サービス(std_srvs/SetBool)経由でも切替可能で、
                                         投入(L4/R4)シーケンス開始時にGUI側が自動で
                                         OFFにする(投入シーケンスは手先θを固定値
                                         shoot_tip_theta_radへ制御するため、追従ONの
                                         ままだと本ノードが毎周期-root_thetaへ上書き
                                         して競合するのを防ぐ、_on_set_tip_theta_
                                         follow_srv参照))
  SHAREボタン      -> XY移動モードトグル (xy_move_toggle_button, デフォルト8。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-03追加、ユーザー指定:「SHAREでX,Y
                                         移動モードに切り替え。現状は各関節の角度を
                                         人が調整しているがこのモードではスティックで
                                         X,Y方向に手先を動かせる」。ONの間は左スティック
                                         左右(axis_x、既定はaxis_thetaと同じ0)・左
                                         スティック上下(axis_y、既定はaxis_rと同じ1)を
                                         root_theta_joint/r_jointの直接ジョグではなく
                                         ワールドXY(command_gui_node.xyz_to_joint/
                                         joint_to_xyzと同じ極座標変換、X軸正=右向き・
                                         Y軸正=前方)へのジョグとして解釈し、逆変換で
                                         target_theta_・target_r_を同時に更新する。
                                         OFFなら従来通りroot_theta/rを別々に直接
                                         ジョグする(関節モード)。2026-09-03、同日
                                         ユーザー指定:「デフォルトの手動操作モードを
                                         XYモードに」により既定ON(起動直後はXY
                                         モード、SHAREを押すと関節モードへ切り替わる。
                                         以前は既定OFF=起動直後は関節モードだった)
  十字キー(D-pad)  -> GUI上の目標ワーク選択カーソル移動 (axis_select_col/
                                         axis_select_row, デフォルト6/7。多くの
                                         Linuxジョイスティックドライバでは十字キーが
                                         axes配列の末尾2要素として出てくる想定、
                                         実機で要確認。2026-09-03追加、ユーザー指定:
                                         「矢印キーでGUI上で目標ワークを選択し移動
                                         バツで移動、再度バツで回収実行」)。左右/
                                         上下いずれも閾値0.5を「押されている」とみなし、
                                         立ち上がりエッジ(押しっぱなしでは連続移動
                                         しない)でcommand_gui_nodeのselect_work_up・
                                         _down・_left・_right(std_srvs/Trigger)の
                                         いずれかを1回だけ呼ぶ。移動系のenable_button
                                         (デッドマン)とは独立に扱う(ポンプトグル
                                         ボタンと同じ理由)。選択カーソルの移動のみで
                                         シーケンス開始は行わない(開始は×ボタン=
                                         pickup_confirm_buttonが選択中ワークへの
                                         移動を、PSボタン=pick_confirm_buttonが
                                         回収実行の確定を、それぞれ別ボタンで担う。
                                         上記×・PSボタンの項目参照)

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
import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

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

# trajectory_follower_node.py/display.launch.pyのINITIAL_ROOT_THETA_RAD/zerosと
# 一致させること(sim起動直後、フィールドに平行・ハンドが右側になる向き、
# 2026-09-03)。
INITIAL_ROOT_THETA_RAD = -math.pi / 2.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        self.declare_parameter('axis_theta', 0)
        # z_jointの手動ジョグ軸(右スティック上下、2026-09-03、同日ユーザー
        # 指定:「Zは右スティック上下で」により左スティック上下から変更。
        # XY移動モード・関節モードいずれでも常にこの軸がz_jointを操作する)。
        self.declare_parameter('axis_z', 4)
        # r_jointの手動ジョグ軸(左スティック上下、2026-09-03、同日z_jointが
        # 右スティック上下(axis_z=4)へ移ったため、そのままだと関節モードで
        # r_jointとz_jointが同時に動いてしまう。競合を避けるため十字キー上下
        # →右スティック上下→左スティック上下、と再度変更した)。
        self.declare_parameter('axis_r', 1)
        # 手先θ(tip_theta_joint)の手動ジョグ軸(2026-09-03追加)。右スティック
        # 左右を想定、-1で無効。
        self.declare_parameter('axis_tip_theta', 3)
        # XY移動モード(xy_move_toggle_button参照、2026-09-03追加)でのワールドXY
        # ジョグ軸。既定はaxis_theta/axis_rと同じ物理スティック、左スティックの
        # 左右・上下を両方使う(2026-09-03、同日ユーザー指定:「XYは左スティック」
        # によりaxis_yを4→1に変更)。
        self.declare_parameter('axis_x', 0)
        self.declare_parameter('axis_y', 1)
        self.declare_parameter('invert_theta', False)
        self.declare_parameter('invert_z', False)
        self.declare_parameter('invert_r', False)
        self.declare_parameter('invert_tip_theta', False)
        # 実機/シミュレータで確認したところXY移動モードのX(左スティック左右)が
        # 逆方向だったため既定Trueに変更(2026-09-03、ユーザー指摘:「左右が
        # 反転している」)。
        self.declare_parameter('invert_x', True)
        self.declare_parameter('invert_y', False)
        # -1ならデッドマンボタン無効(常時有効)。実機ではボタンを割り当てて
        # 誤操作による意図しない動作を防ぐことを推奨。
        self.declare_parameter('enable_button', -1)
        self.declare_parameter('theta_speed', 1.0)   # rad/s (フル入力時)
        self.declare_parameter('z_speed', 0.2)       # m/s (フル入力時)
        self.declare_parameter('r_speed', 0.2)       # m/s (フル入力時)
        self.declare_parameter('tip_theta_speed', 1.0)  # rad/s (フル入力時、2026-09-03追加)
        self.declare_parameter('xy_speed', 0.2)         # m/s (フル入力時、2026-09-03追加)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('update_rate_hz', 50.0)
        # ポンプON/OFFトグル用ボタン(2026-09-03追加、同日△ボタンへ再割当)。
        # -1ならボタン操作無効。
        self.declare_parameter('pump_toggle_button', 2)
        # 選択中ワークへの移動ボタン(×ボタン、2026-09-03追加)。-1ならボタン
        # 操作無効。立ち上がりエッジ即時でpick_sequence_moveを呼ぶだけの単純な
        # ボタン(2026-09-03、同日一時的に「長押しで回収実行確定」もこのボタンに
        # 統合していたが、移動が実行中シーケンスを即座に中断・やり直す仕様に
        # なったため長押しのつもりで押した瞬間に移動が先に発火し回収実行待ち
        # 状態を壊してしまう不具合が発生。ユーザー指定:「回収ボタンをバツ長押し
        # からPSボタンに変更」により確定は下記pick_confirm_buttonへ分離した)。
        self.declare_parameter('pickup_confirm_button', 0)
        # 回収実行(確定)ボタン(PSボタン、2026-09-03追加)。-1ならボタン操作無効。
        # 立ち上がりエッジ即時でpick_sequence_confirmを呼ぶだけの単純なボタン。
        # 誤操作でワークに接触・吸着してしまうことを防ぐため、移動用の
        # pickup_confirm_buttonとは意図的に別ボタンにしている。
        self.declare_parameter('pick_confirm_button', 10)
        # L4/R4へ移動ボタン(2026-09-03追加、同日□/○ボタンへ再割当)。
        # -1ならボタン操作無効。
        self.declare_parameter('shoot_start_l4_button', 3)
        self.declare_parameter('shoot_start_r4_button', 1)
        # 手先θのroot_theta追従トグルボタン(2026-09-03追加)。OPTIONSボタンを
        # 想定。-1ならボタン操作無効(常にトグル状態は初期値のまま変わらない)。
        self.declare_parameter('tip_theta_follow_theta_button', 9)
        # XY移動モードトグルボタン(2026-09-03追加)。SHAREボタンを想定。
        # -1ならボタン操作無効。
        self.declare_parameter('xy_move_toggle_button', 8)
        # GUI上のワーク選択カーソル移動軸(十字キー、2026-09-03追加)。多くの
        # Linuxジョイスティックドライバでは十字キーがaxes配列の末尾2要素として
        # 出てくる想定。-1で該当方向を無効。
        self.declare_parameter('axis_select_col', 6)   # 十字キー左右
        self.declare_parameter('axis_select_row', 7)   # 十字キー上下
        self.declare_parameter('invert_select_col', False)
        self.declare_parameter('invert_select_row', False)

        self.axis_theta_ = int(self.get_parameter('axis_theta').value)
        self.axis_z_ = int(self.get_parameter('axis_z').value)
        self.axis_r_ = int(self.get_parameter('axis_r').value)
        self.axis_tip_theta_ = int(self.get_parameter('axis_tip_theta').value)
        self.axis_x_ = int(self.get_parameter('axis_x').value)
        self.axis_y_ = int(self.get_parameter('axis_y').value)
        self.axis_select_col_ = int(self.get_parameter('axis_select_col').value)
        self.axis_select_row_ = int(self.get_parameter('axis_select_row').value)
        self.sign_theta_ = -1.0 if self.get_parameter('invert_theta').value else 1.0
        self.sign_z_ = -1.0 if self.get_parameter('invert_z').value else 1.0
        self.sign_r_ = -1.0 if self.get_parameter('invert_r').value else 1.0
        self.sign_tip_theta_ = -1.0 if self.get_parameter('invert_tip_theta').value else 1.0
        self.sign_x_ = -1.0 if self.get_parameter('invert_x').value else 1.0
        self.sign_y_ = -1.0 if self.get_parameter('invert_y').value else 1.0
        self.sign_select_col_ = -1.0 if self.get_parameter('invert_select_col').value else 1.0
        self.sign_select_row_ = -1.0 if self.get_parameter('invert_select_row').value else 1.0
        self.enable_button_ = int(self.get_parameter('enable_button').value)
        self.theta_speed_ = float(self.get_parameter('theta_speed').value)
        self.z_speed_ = float(self.get_parameter('z_speed').value)
        self.r_speed_ = float(self.get_parameter('r_speed').value)
        self.tip_theta_speed_ = float(self.get_parameter('tip_theta_speed').value)
        self.xy_speed_ = float(self.get_parameter('xy_speed').value)
        self.deadzone_ = float(self.get_parameter('deadzone').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.dt_ = 1.0 / update_rate_hz
        self.pump_toggle_button_ = int(self.get_parameter('pump_toggle_button').value)
        self.pickup_confirm_button_ = int(self.get_parameter('pickup_confirm_button').value)
        self.pick_confirm_button_ = int(self.get_parameter('pick_confirm_button').value)
        self.shoot_start_l4_button_ = int(self.get_parameter('shoot_start_l4_button').value)
        self.shoot_start_r4_button_ = int(self.get_parameter('shoot_start_r4_button').value)
        self.tip_theta_follow_theta_button_ = int(
            self.get_parameter('tip_theta_follow_theta_button').value)
        self.xy_move_toggle_button_ = int(self.get_parameter('xy_move_toggle_button').value)

        # 現在の目標関節角度(スティック/十字キー入力をここへ積分していく)。
        # /mixed_joint_statesを受信するまでは、trajectory_follower_node起動直後の
        # 実際の静止姿勢に合わせておく。ここを可動域の中央など実際と異なる値に
        # すると、/mixed_joint_states受信前に一部の軸だけ操作した場合でも同じ
        # JointStateメッセージに含まれる他の軸がその適当な初期値へ動いてしまう。
        self.target_theta_ = INITIAL_ROOT_THETA_RAD
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

        # ポンプON/OFFトグル(△ボタン、2026-09-03追加)。現在のON/OFF状態は
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

        # 選択中ワークへの移動ボタン(×ボタン、2026-09-03追加)。立ち上がり
        # エッジ即時でpick_sequence_moveを呼ぶだけの単純なボタン(同日、一時
        # 「長押しで回収実行確定」も統合していたが不具合が出たため撤回、
        # pickup_confirm_button宣言部のコメント参照)。
        self._prev_pickup_move_pressed_ = False
        self._pickup_move_client_ = self.create_client(Trigger, 'pick_sequence_move')

        # 回収実行(確定)ボタン(PSボタン、2026-09-03追加)。立ち上がりエッジ
        # 即時でpick_sequence_confirmを呼ぶだけの単純なボタン。
        # command_gui_node側の回収シーケンスが「ワーク手前で自動停止→回収実行
        # 待ち」のときに続行させる。
        self._prev_pick_confirm_button_pressed_ = False
        self._pickup_confirm_client_ = self.create_client(Trigger, 'pick_sequence_confirm')

        # L4/R4へ移動ボタン(□/○ボタン、2026-09-03追加)。command_gui_node側の
        # /shoot_sequence_start_l4・_r4サービスを呼び、固定のシューティングエリア
        # (L4/R4)へ向かわせる。
        self._prev_shoot_start_l4_pressed_ = False
        self._shoot_start_l4_client_ = self.create_client(Trigger, 'shoot_sequence_start_l4')
        self._prev_shoot_start_r4_pressed_ = False
        self._shoot_start_r4_client_ = self.create_client(Trigger, 'shoot_sequence_start_r4')

        # 手先θのroot_theta追従トグル(OPTIONSボタン、2026-09-03追加)。既定でON
        # (ユーザー指定:「デフォルトを自動シーケンスで実行に」)。ONの間は
        # _timer_callbackがtip_theta_joint = -root_theta_jointを毎周期指令し、
        # 右スティック左右の手動ジョグ入力は無視する(_update_tip_theta_follow_
        # toggle参照)。
        self._tip_theta_follow_theta_ = True
        self._prev_follow_toggle_pressed_ = False
        # command_gui_node側からも明示的にON/OFFできるサービス(2026-09-03追加、
        # ユーザー指定:「手先θ追従はシューティングボックスへの自動移動時には
        # 自動で無効化」)。投入(L4/R4)シーケンス開始時にGUI側がこれを呼んで
        # OFFにする(_on_set_tip_theta_follow_srv参照。投入シーケンスは手先θを
        # 固定値shoot_tip_theta_radへ制御するため、追従ONのままだと本ノードが
        # 毎周期-root_thetaへ上書きして競合するのを防ぐ)。
        self.create_service(
            SetBool, 'set_tip_theta_follow_theta', self._on_set_tip_theta_follow_srv)

        # XY移動モード(SHAREボタン、2026-09-03追加)。ONの間はaxis_x/axis_yの
        # 入力をワールドXYのジョグとして解釈し、target_theta_・target_r_を
        # 同時に更新する(_timer_callback参照。ユーザー指定:「SHAREでX,Y移動
        # モードに切り替え」)。既定ON(2026-09-03、同日ユーザー指定:「デフォルト
        # の手動操作モードをXYモードに」。以前は既定OFF=起動直後はroot_theta/r
        # を別々に直接ジョグする関節モードだった。SHAREを押すと関節モードへ
        # 切り替わる)。
        self._xy_move_mode_ = True
        self._prev_xy_move_toggle_pressed_ = False

        # 十字キー(D-pad)でのGUIワーク選択カーソル移動(2026-09-03追加、
        # ユーザー指定:「矢印キーでGUI上で目標ワークを選択し移動バツで移動、
        # 再度バツで回収実行」)。command_gui_node側のselect_work_up・_down・
        # _left・_rightサービス(std_srvs/Trigger)を、十字キーの立ち上がり
        # エッジ(押しっぱなしでは連続移動しない)で1回だけ呼ぶ。
        # _prev_select_col_pressed_/_prev_select_row_pressed_で左右/上下を
        # 別々にエッジ検出する(十字キーが2軸で同時に効くドライバでも同時押し
        # 斜め入力で1周期に最大2回(上下+左右)まで呼べるようにするため)。
        self._prev_select_col_pressed_ = False
        self._prev_select_row_pressed_ = False
        self._select_work_up_client_ = self.create_client(Trigger, 'select_work_up')
        self._select_work_down_client_ = self.create_client(Trigger, 'select_work_down')
        self._select_work_left_client_ = self.create_client(Trigger, 'select_work_left')
        self._select_work_right_client_ = self.create_client(Trigger, 'select_work_right')

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
            f'tip_theta_follow_theta_button={self.tip_theta_follow_theta_button_} '
            f'(follow={self._tip_theta_follow_theta_}), '
            f'xy_move_toggle_button={self.xy_move_toggle_button_}, xy_speed={self.xy_speed_}m/s, '
            f'axis_select_col={self.axis_select_col_}, axis_select_row={self.axis_select_row_}, '
            f'pickup_confirm_button={self.pickup_confirm_button_}, '
            f'pick_confirm_button={self.pick_confirm_button_}, '
            f'rate={update_rate_hz}Hz')

    def _on_set_parameters(self, params):
        for p in params:
            if (p.name in ('theta_speed', 'z_speed', 'r_speed', 'tip_theta_speed', 'xy_speed')
                    and p.value <= 0.0):
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
            elif p.name == 'xy_speed':
                self.xy_speed_ = float(p.value)
        return SetParametersResult(successful=True)

    def _on_joy(self, msg: Joy):
        self.latest_joy_ = msg

    def _on_pump_state(self, msg: Bool):
        self._pump_on_ = msg.data

    def _update_pump_toggle(self, msg: Joy):
        """△ボタンの立ち上がりエッジで、hand_nodeから購読済みの現在のポンプ状態と
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

    def _update_pickup_move(self, msg: Joy):
        """×ボタンの立ち上がりエッジで、command_gui_nodeの/pick_sequence_move
        (選択中ワークへの移動のみ)を呼ぶ(std_srvs/Trigger、2026-09-03追加)。
        他のシーケンス実行中(回収実行待ち・投入シーケンス中含む)でも、
        command_gui_node側が確認や中断操作なしに即座に中断して選択中ワークへ
        切り替える(command_gui_node._start_pick_sequence参照)。
        2026-09-03、同日一時的に「長押しで回収実行確定」もこの×ボタンに統合して
        いたが、上記の「移動が実行中シーケンスを即座に中断・やり直す」仕様のため、
        長押しのつもりで押した瞬間に移動が先に発火して回収実行待ち状態を壊して
        しまう不具合(「バツ長押しで回収ができなくなった」)が発生した。ユーザー
        指定:「回収ボタンをバツ長押しからPSボタンに変更」により、確定は
        別ボタン(_update_pick_confirm_button、PSボタン)へ分離し、こちらは単純な
        即時押下のみに戻した。移動系のenable_button(デッドマン)とは独立に扱う
        (ポンプトグルボタンと同じ理由)。"""
        if self.pickup_confirm_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.pickup_confirm_button_ < len(buttons)
                   and bool(buttons[self.pickup_confirm_button_]))
        if pressed and not self._prev_pickup_move_pressed_:
            self._call_trigger(self._pickup_move_client_, 'ワーク移動(pick_sequence_move)')
        self._prev_pickup_move_pressed_ = pressed

    def _update_pick_confirm_button(self, msg: Joy):
        """PSボタンの立ち上がりエッジで、command_gui_nodeの/pick_sequence_confirm
        (回収実行の確定のみ)を呼ぶ(std_srvs/Trigger、2026-09-03追加、ユーザー
        指定:「回収ボタンをバツ長押しからPSボタンに変更」)。「ワーク手前で
        自動停止→回収実行待ち」の状態でなければcommand_gui_node側が失敗を
        返すだけ(安全側、意図せぬ確定=接触・吸着を防ぐためのガード)。誤操作を
        防ぐため、移動用の×ボタン(_update_pickup_move)とは意図的に別ボタンに
        している。移動系のenable_button(デッドマン)とは独立に扱う(ポンプ
        トグルボタンと同じ理由)。"""
        if self.pick_confirm_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.pick_confirm_button_ < len(buttons)
                   and bool(buttons[self.pick_confirm_button_]))
        if pressed and not self._prev_pick_confirm_button_pressed_:
            self._call_trigger(self._pickup_confirm_client_, '回収実行確定(pick_sequence_confirm)')
        self._prev_pick_confirm_button_pressed_ = pressed

    def _update_shoot_start_l4(self, msg: Joy):
        """□ボタンの立ち上がりエッジで、command_gui_nodeの
        /shoot_sequence_start_l4(std_srvs/Trigger)を呼び、シューティングエリア
        L4へ向かわせる(2026-09-03追加)。移動系のenable_button(デッドマン)とは
        独立に扱う(ポンプトグルボタンと同じ理由)。他のシーケンスが実行中でも
        command_gui_node側が確認や中断操作なしに即座に中断してこちらへ切り替える
        (2026-09-03、ユーザー指摘:「回収実行を押さなくてもシューティング位置へ
        移動できるように。ユーザーの動きを制限したくない」。以前はここで
        「完了後に自動開始する予約」扱いだったが、即座に切り替えられるように
        なったため不要になった、_on_shoot_start_requested参照)。"""
        if self.shoot_start_l4_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.shoot_start_l4_button_ < len(buttons)
                   and bool(buttons[self.shoot_start_l4_button_]))
        if pressed and not self._prev_shoot_start_l4_pressed_:
            if self._shoot_start_l4_client_.service_is_ready():
                self._shoot_start_l4_client_.call_async(Trigger.Request())
            else:
                self.get_logger().warning(
                    'joy_teleop_node: command_gui_nodeのL4移動サービスに接続できません(GUI未起動?)')
        self._prev_shoot_start_l4_pressed_ = pressed

    def _update_shoot_start_r4(self, msg: Joy):
        """○ボタンの立ち上がりエッジで、command_gui_nodeの
        /shoot_sequence_start_r4(std_srvs/Trigger)を呼び、シューティングエリア
        R4へ向かわせる(2026-09-03追加)。移動系のenable_button(デッドマン)とは
        独立に扱う(ポンプトグルボタンと同じ理由)。他のシーケンスが実行中でも
        command_gui_node側が確認や中断操作なしに即座に中断してこちらへ切り替える
        (L4の項目と同じ理由、_on_shoot_start_requested参照)。"""
        if self.shoot_start_r4_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.shoot_start_r4_button_ < len(buttons)
                   and bool(buttons[self.shoot_start_r4_button_]))
        if pressed and not self._prev_shoot_start_r4_pressed_:
            if self._shoot_start_r4_client_.service_is_ready():
                self._shoot_start_r4_client_.call_async(Trigger.Request())
            else:
                self.get_logger().warning(
                    'joy_teleop_node: command_gui_nodeのR4移動サービスに接続できません(GUI未起動?)')
        self._prev_shoot_start_r4_pressed_ = pressed

    def _update_tip_theta_follow_toggle(self, msg: Joy):
        """OPTIONSボタンの立ち上がりエッジで、手先θ(tip_theta_joint)をroot_theta
        に追従させるかどうかのローカルなトグル状態を反転する(2026-09-03追加、
        ユーザー指定: 「OPTIONSボタンで手先θを根本θに追従させるか切り替え
        られるように」)。他のボタンと違いサービス呼び出しは行わず、
        self._tip_theta_follow_theta_を直接書き換えるだけ(_timer_callback側の
        tip_theta_joint計算が参照する)。移動系のenable_button(デッドマン)とは
        独立に扱う(ポンプトグルボタンと同じ理由)。"""
        if self.tip_theta_follow_theta_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.tip_theta_follow_theta_button_ < len(buttons)
                   and bool(buttons[self.tip_theta_follow_theta_button_]))
        if pressed and not self._prev_follow_toggle_pressed_:
            self._tip_theta_follow_theta_ = not self._tip_theta_follow_theta_
            self.get_logger().info(
                'joy_teleop_node: 手先θのroot_theta追従を'
                f'{"ON" if self._tip_theta_follow_theta_ else "OFF"}にしました')
        self._prev_follow_toggle_pressed_ = pressed

    def _on_set_tip_theta_follow_srv(self, request, response):
        """command_gui_node側から手先θのroot_theta追従を明示的にON/OFFする
        (std_srvs/SetBool、2026-09-03追加)。OPTIONSボタンのトグルと同じ
        self._tip_theta_follow_theta_を直接書き換えるだけ。投入(L4/R4)
        シーケンス開始時にGUI側がFalseを渡して自動でOFFにするために使う
        (ユーザー指定:「手先θ追従はシューティングボックスへの自動移動時には
        自動で無効化」。投入シーケンスは手先θを固定値shoot_tip_theta_radへ
        制御するため、追従ONのままだと本ノードが毎周期-root_thetaへ上書き
        して競合する)。"""
        self._tip_theta_follow_theta_ = bool(request.data)
        self.get_logger().info(
            'joy_teleop_node: 手先θのroot_theta追従を'
            f'{"ON" if self._tip_theta_follow_theta_ else "OFF"}にしました(GUI経由)')
        response.success = True
        response.message = f'tip_theta_follow_theta={self._tip_theta_follow_theta_}'
        return response

    def _update_xy_move_toggle(self, msg: Joy):
        """SHAREボタンの立ち上がりエッジで、XY移動モードのローカルなトグル状態を
        反転する(2026-09-03追加、ユーザー指定:「SHAREでX,Y移動モードに切り替え。
        現状は各関節の角度を人が調整しているがこのモードではスティックでX,Y方向に
        手先を動かせる」)。self._xy_move_mode_を直接書き換えるだけ(_timer_callback
        側のtheta/r計算が参照する)。移動系のenable_button(デッドマン)とは独立に
        扱う(ポンプトグルボタンと同じ理由)。"""
        if self.xy_move_toggle_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.xy_move_toggle_button_ < len(buttons)
                   and bool(buttons[self.xy_move_toggle_button_]))
        if pressed and not self._prev_xy_move_toggle_pressed_:
            self._xy_move_mode_ = not self._xy_move_mode_
            self.get_logger().info(
                'joy_teleop_node: XY移動モードを'
                f'{"ON" if self._xy_move_mode_ else "OFF"}にしました')
        self._prev_xy_move_toggle_pressed_ = pressed

    def _call_trigger(self, client, description):
        if client.service_is_ready():
            client.call_async(Trigger.Request())
        else:
            self.get_logger().warning(
                f'joy_teleop_node: command_gui_nodeの{description}サービスに接続できません'
                '(GUI未起動?)')

    def _update_work_selection(self, msg: Joy):
        """十字キー(D-pad)の立ち上がりエッジで、GUI上のワーク選択カーソルを
        1マス動かすTriggerサービス(select_work_up/down/left/right)を呼ぶ
        (2026-09-03追加、ユーザー指定:「矢印キーでGUI上で目標ワークを選択し
        移動」)。左右(axis_select_col)・上下(axis_select_row)を別々にエッジ
        検出し、|値|>0.5を「押されている」とみなす(多くのLinuxドライバでは
        十字キーは-1/0/1の離散値で出てくる想定だが、閾値判定にしておけば
        連続値で出てくるドライバでも動く)。押しっぱなしで連続移動はしない
        (離してから再度倒す必要がある)。移動系のenable_button(デッドマン)とは
        独立に扱う(ポンプトグルボタンと同じ理由)。
        実機/シミュレータで確認したところ、col_val>0が右ではなく左に対応して
        いた(2026-09-03、ユーザー指摘:「左右が反転している」)ため、下記の
        left/rightクライアントの対応をここで入れ替えている(invert_select_col
        パラメータは符号反転用に別途用意してあるが、今回の食い違いは値の符号
        ではなく対応関係そのものだったため、こちらで吸収した)。"""
        if self.axis_select_col_ >= 0:
            col_val = self._axis(msg.axes, self.axis_select_col_) * self.sign_select_col_
            col_pressed = abs(col_val) > 0.5
            if col_pressed and not self._prev_select_col_pressed_:
                client = (self._select_work_left_client_ if col_val > 0
                          else self._select_work_right_client_)
                self._call_trigger(client, 'ワーク選択(左右)')
            self._prev_select_col_pressed_ = col_pressed

        if self.axis_select_row_ >= 0:
            row_val = self._axis(msg.axes, self.axis_select_row_) * self.sign_select_row_
            row_pressed = abs(row_val) > 0.5
            if row_pressed and not self._prev_select_row_pressed_:
                client = (self._select_work_up_client_ if row_val > 0
                          else self._select_work_down_client_)
                self._call_trigger(client, 'ワーク選択(上下)')
            self._prev_select_row_pressed_ = row_pressed

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
        self._update_pickup_move(msg)
        self._update_pick_confirm_button(msg)
        self._update_shoot_start_l4(msg)
        self._update_shoot_start_r4(msg)
        self._update_tip_theta_follow_toggle(msg)
        self._update_xy_move_toggle(msg)
        self._update_work_selection(msg)
        enabled = self._is_enabled(msg)

        theta_in = apply_deadzone(self._axis(msg.axes, self.axis_theta_), self.deadzone_) * self.sign_theta_
        z_in = apply_deadzone(self._axis(msg.axes, self.axis_z_), self.deadzone_) * self.sign_z_
        r_in = apply_deadzone(self._axis(msg.axes, self.axis_r_), self.deadzone_) * self.sign_r_
        tip_theta_in = (apply_deadzone(self._axis(msg.axes, self.axis_tip_theta_), self.deadzone_)
                        * self.sign_tip_theta_)
        x_in = apply_deadzone(self._axis(msg.axes, self.axis_x_), self.deadzone_) * self.sign_x_
        y_in = apply_deadzone(self._axis(msg.axes, self.axis_y_), self.deadzone_) * self.sign_y_

        names = []
        positions = []

        # レート方式。入力が中立/デッドマン未押下の間は目標を現在値に同期する
        # だけでpublishに含めない(離した位置を保持する。autoモードの動作を
        # 妨げず、モード切替時の急変も防ぐ)。
        if enabled and z_in != 0.0:
            self.target_z_ = clamp(self.target_z_ + z_in * self.z_speed_ * self.dt_, Z_LOWER, Z_UPPER)
            names.append('z_joint')
            positions.append(self.target_z_)
        elif self.has_current_state_:
            self.target_z_ = self._current_z_

        if self._xy_move_mode_:
            # SHAREボタンでトグル(既定ON、_update_xy_move_toggle参照)の間は、
            # axis_theta/axis_rを直接ジョグする代わりに、axis_x/axis_y(既定は
            # 同じ物理スティック)をワールドXYのジョグとして解釈する
            # (command_gui_node.xyz_to_joint/joint_to_xyzと同じ極座標変換、
            # X軸正=右向き・Y軸正=前方。ユーザー指定:「SHAREでX,Y移動モードに
            # 切り替え。このモードではスティックでX,Y方向に手先を動かせる」)。
            if enabled and (x_in != 0.0 or y_in != 0.0):
                radius = self.target_r_ + ARM_LENGTH / 2.0
                x = -radius * math.sin(self.target_theta_) + x_in * self.xy_speed_ * self.dt_
                y = radius * math.cos(self.target_theta_) + y_in * self.xy_speed_ * self.dt_
                self.target_theta_ = clamp(math.atan2(-x, y), ROOT_THETA_LOWER, ROOT_THETA_UPPER)
                self.target_r_ = clamp(math.hypot(x, y) - ARM_LENGTH / 2.0, R_LOWER, R_UPPER)
                names.append('root_theta_joint')
                positions.append(self.target_theta_)
                names.append('r_joint')
                positions.append(self.target_r_)
            elif self.has_current_state_:
                self.target_theta_ = self._current_theta_
                self.target_r_ = self._current_r_
        else:
            if enabled and theta_in != 0.0:
                self.target_theta_ = clamp(
                    self.target_theta_ + theta_in * self.theta_speed_ * self.dt_,
                    ROOT_THETA_LOWER, ROOT_THETA_UPPER)
                names.append('root_theta_joint')
                positions.append(self.target_theta_)
            elif self.has_current_state_:
                self.target_theta_ = self._current_theta_

            if enabled and r_in != 0.0:
                self.target_r_ = clamp(
                    self.target_r_ + r_in * self.r_speed_ * self.dt_, R_LOWER, R_UPPER)
                names.append('r_joint')
                positions.append(self.target_r_)
            elif self.has_current_state_:
                self.target_r_ = self._current_r_

        # tip_theta_jointはcontinuous(可動域制限なし)なのでclampしない。
        # trajectory_follower_nodeがtip_theta_joint未構成の場合はpublishしても
        # target_callback側で無視されるだけなので、has_tip_theta_state_の有無に
        # 関わらず常に試みる(2026-09-03追加)。
        if self._tip_theta_follow_theta_:
            # OPTIONSボタンでON(既定ON、_update_tip_theta_follow_toggle参照)の
            # 間は、回収シーケンスと同じ追従式(tip_theta=-root_theta、ワークの
            # 行と平行を保つ)を手動ジョグ中も毎周期指令し続ける。右スティック
            # 左右(tip_theta_in)による独立ジョグはこの間無視する。
            self.target_tip_theta_ = -self.target_theta_
            names.append('tip_theta_joint')
            positions.append(self.target_tip_theta_)
        elif enabled and tip_theta_in != 0.0:
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
