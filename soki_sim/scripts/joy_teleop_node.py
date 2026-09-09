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
  左スティック左右 -> X(XYモード時)/root_theta_joint(関節モード時、既定
                                         theta_jog_enabled=falseのため実際には
                                         無効。2026-09-09、manualブランチでの
                                         操作方針「根本θのみ自動で位置合わせ、
                                         RとZは人が速度制御」によりroot_thetaの
                                         直接ジョグは既定で無効化し、command_gui_
                                         nodeの既存の回収/投入シーケンスによる
                                         自動位置合わせに任せている。
                                         theta_jog_enabled=trueで手動ジョグへ
                                         戻せる。axis_x/axis_theta、いずれも
                                         デフォルト0。下記SHAREボタンの項目参照)
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
  ×(バツ)ボタン   -> 選択中ワークへ回収シーケンス開始 (pickup_confirm_button,
                                         デフォルト0。PS4/PS5コントローラの
                                         一般的なLinuxドライバ割り当てを仮定した値、
                                         実機で要確認。2026-09-03追加、2026-09-09
                                         仕様変更(Z軸自動降下の廃止により「移動」
                                         「確定」の2段階が不要になり1つに統合)。
                                         立ち上がりエッジ即時でcommand_gui_nodeの
                                         /pick_sequence_moveサービスを呼び、
                                         矢印キー(D-pad)で選択中のワークへハンド
                                         展開・ポンプON・theta回転のみの回収
                                         シーケンスを開始する(GUIの「回収実行」
                                         ボタンと同じ効果。他のシーケンス実行中
                                         でも即座に中断して切り替わる)。R/Z軸は
                                         人がjoyの速度指令モードで操作する)
  十字キー(D-pad)  -> GUI上の目標ワーク選択カーソル移動 (axis_select_col/
                                         axis_select_row, デフォルト6/7。多くの
                                         Linuxジョイスティックドライバでは十字キーが
                                         axes配列の末尾2要素として出てくる想定、
                                         実機で要確認。2026-09-03追加、ユーザー指定:
                                         「矢印キーでGUI上で目標ワークを選択し移動
                                         バツで移動」)。左右/上下いずれも閾値0.5を
                                         「押されている」とみなし、立ち上がりエッジ
                                         (押しっぱなしでは連続移動しない)で
                                         command_gui_nodeのselect_work_up・_down・
                                         _left・_right(std_srvs/Trigger)のいずれかを
                                         1回だけ呼ぶ。移動系のenable_button
                                         (デッドマン)とは独立に扱う(ポンプトグル
                                         ボタンと同じ理由)。選択カーソルの移動のみで
                                         シーケンス開始は行わない(開始は×ボタン=
                                         pickup_confirm_buttonが担う。上記×ボタンの
                                         項目参照)
  PSボタン         -> ソフト緊急停止   (estop_button, デフォルト10。PS4/PS5
                                         コントローラの一般的なLinuxドライバ
                                         割り当てを仮定した値、実機で要確認。
                                         2026-09-08追加。立ち上がりエッジ即時で
                                         command_gui_nodeの/emergency_stop
                                         サービスを呼ぶ(GUIの「緊急停止」ボタンと
                                         同じ効果。trajectory_follower_nodeの
                                         cubemars/robomas出力を凍結し、GUIの自動
                                         シーケンスも中断する。解除はGUI側の
                                         「解除」ボタンのみ(誤操作で即再始動しない
                                         よう、ボタン一つでは解除できない設計)。
                                         移動系のenable_button(デッドマン)とは
                                         独立に扱う(安全機能のため常に効くように
                                         する))
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
                                         ジョグする(関節モード)。既定OFF(起動直後は
                                         関節モード、SHAREを押すとXYモードへ切り替わる。
                                         2026-09-03に一度既定ONへ変更したが、
                                         2026-09-09、手動移動にフォーカスするmanual
                                         ブランチでの方針変更によりデフォルトの移動
                                         モードを速度指令(velocity_mode_enabled)に
                                         した際、速度指令モードがXYモード中のr_joint/
                                         thetaを対象外にしているため関節モードへ
                                         再度デフォルトを戻した)

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
        # z/rのjoy出力を位置目標(target_z_/target_r_を積分してjoint_targetsへ)
        # ではなく、trajectory_follower_nodeの速度モード(robomas_velocity_mode)
        # 向けにスティック入力をそのまま速度指令(joint_velocity_targets)として
        # 送るモード(2026-09-09追加、command_gui_nodeの「joy速度指令モード」
        # チェックボックスから切り替える想定)。XY移動モード中はr_joint/theta側は
        # このモードの対象外(XY変換のみ、_timer_callback参照)。ONの間、z_speed_/
        # r_speed_はそのまま「フル入力時の速度[m/s]」としてtarget_z_/target_r_の
        # 積分ではなく直接の速度指令値に使う(既存のレート方式と単位を揃えるため
        # 新規パラメータは追加しない)。既定true(2026-09-09、手動移動にフォーカス
        # するmanualブランチでの方針変更によりデフォルトの移動モードを速度指令へ。
        # trajectory_follower_node側のrobomas_velocity_modeと揃えること)。
        self.declare_parameter('velocity_mode_enabled', True)
        # root_theta_jointの手動joyジョグ(左スティック左右、axis_theta)を許可するか
        # (2026-09-09追加、manualブランチでの操作方針:「根本θのみ自動で位置合わせ
        # （手先θは一旦放置）、RとZは人が速度制御で操作」。root_thetaはcommand_gui_
        # nodeの既存の回収/投入シーケンス(pick_sequence_move等、×/R2/□/○ボタン)に
        # よる自動位置合わせに任せ、joyからは直接動かさない方針のため既定false。
        # 手動ジョグへ戻したい場合用にパラメータとして残してある)。
        self.declare_parameter('theta_jog_enabled', False)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('update_rate_hz', 50.0)
        # ポンプON/OFFトグル用ボタン(2026-09-03追加、同日△ボタンへ再割当)。
        # -1ならボタン操作無効。
        self.declare_parameter('pump_toggle_button', 2)
        # 選択中ワークへ回収シーケンスを開始するボタン(×ボタン、2026-09-03追加、
        # 2026-09-09回収シーケンス復元にあわせて仕様変更)。-1ならボタン操作無効。
        # 立ち上がりエッジ即時でcommand_gui_nodeの/pick_sequence_moveを呼ぶだけの
        # 単純なボタン(以前あった「移動」「確定(回収実行)」の2段階は、Z軸自動
        # 降下が無くなったため不要になり1つに統合した)。
        self.declare_parameter('pickup_confirm_button', 0)
        # ソフト緊急停止ボタン(PSボタン、2026-09-08追加)。-1ならボタン操作無効。
        # 立ち上がりエッジ即時でcommand_gui_nodeの/emergency_stopサービスを呼ぶ
        # (_update_estop_button参照)。
        self.declare_parameter('estop_button', 10)
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
        self.velocity_mode_enabled_ = bool(self.get_parameter('velocity_mode_enabled').value)
        self.theta_jog_enabled_ = bool(self.get_parameter('theta_jog_enabled').value)
        self.deadzone_ = float(self.get_parameter('deadzone').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.dt_ = 1.0 / update_rate_hz
        self.pump_toggle_button_ = int(self.get_parameter('pump_toggle_button').value)
        self.pickup_confirm_button_ = int(self.get_parameter('pickup_confirm_button').value)
        self.estop_button_ = int(self.get_parameter('estop_button').value)
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
        # velocity_mode_enabled中、z/rの速度指令をtrajectory_follower_nodeへ送る
        # (joint_targetsとは別トピック、trajectory_follower_node.pyの
        # _on_velocity_targets参照)。
        self.vel_pub_ = self.create_publisher(JointState, 'joint_velocity_targets', 10)
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

        # 選択中ワークへの回収シーケンス開始ボタン(×ボタン、2026-09-03追加、
        # 2026-09-09仕様変更。declare_parameter部コメント参照)。立ち上がりエッジ
        # 即時でcommand_gui_nodeの/pick_sequence_moveを呼ぶだけの単純なボタン。
        self._prev_pickup_move_pressed_ = False
        self._pickup_move_client_ = self.create_client(Trigger, 'pick_sequence_move')

        # ソフト緊急停止ボタン(PSボタン、2026-09-08追加)。立ち上がりエッジ即時で
        # command_gui_nodeの/emergency_stopを呼ぶ(_update_estop_button参照)。
        self._prev_estop_button_pressed_ = False
        self._estop_client_ = self.create_client(Trigger, 'emergency_stop')

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
        # モードに切り替え」)。既定OFF=起動直後はroot_theta/rを別々に直接
        # ジョグする関節モード(2026-09-09、手動移動にフォーカスするmanual
        # ブランチでの方針変更により関節モードへ再度デフォルトを戻した。
        # 2026-09-03に一度既定ONへ変更していたが、速度指令モード(velocity_
        # mode_enabled)がXY移動モード中のr_joint/thetaを対象外にしている
        # ため、速度指令モードをデフォルトにする以上、関節モードもデフォルトに
        # しないとr軸が速度指令の対象外のままになってしまう)。SHAREを押すと
        # XY移動モードへ切り替わる。
        self._xy_move_mode_ = False
        self._prev_xy_move_toggle_pressed_ = False

        # 十字キー(D-pad)でのGUIワーク選択カーソル移動(2026-09-03追加、
        # ユーザー指定:「矢印キーでGUI上で目標ワークを選択し移動」)。
        # command_gui_node側のselect_work_up・_down・_left・_rightサービス
        # (std_srvs/Trigger)を、十字キーの立ち上がりエッジ(押しっぱなしでは
        # 連続移動しない)で1回だけ呼ぶ。_prev_select_col_pressed_/
        # _prev_select_row_pressed_で左右/上下を別々にエッジ検出する(十字キーが
        # 2軸で同時に効くドライバでも同時押し斜め入力で1周期に最大2回(上下+
        # 左右)まで呼べるようにするため)。
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
            f'estop_button={self.estop_button_}, '
            f'velocity_mode_enabled={self.velocity_mode_enabled_}, '
            f'theta_jog_enabled={self.theta_jog_enabled_}, '
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
            elif p.name == 'velocity_mode_enabled':
                self.velocity_mode_enabled_ = bool(p.value)
            elif p.name == 'theta_jog_enabled':
                self.theta_jog_enabled_ = bool(p.value)
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
        (選択中ワークへの回収シーケンス開始)を呼ぶ(std_srvs/Trigger、
        2026-09-03追加、2026-09-09仕様変更)。他のシーケンス実行中でも
        command_gui_node側が確認や中断操作なしに即座に中断して選択中ワークへ
        切り替える(command_gui_node._start_pick_sequence参照)。移動系の
        enable_button(デッドマン)とは独立に扱う(ポンプトグルボタンと同じ理由)。"""
        if self.pickup_confirm_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.pickup_confirm_button_ < len(buttons)
                   and bool(buttons[self.pickup_confirm_button_]))
        if pressed and not self._prev_pickup_move_pressed_:
            self._call_trigger(self._pickup_move_client_, 'ワーク移動(pick_sequence_move)')
        self._prev_pickup_move_pressed_ = pressed

    def _update_estop_button(self, msg: Joy):
        """PSボタンの立ち上がりエッジで、command_gui_nodeの/emergency_stop
        (std_srvs/Trigger、2026-09-08追加)を呼ぶ。GUIの「緊急停止」ボタンと同じ
        効果(trajectory_follower_nodeの出力凍結・ホーミング中断・自動シーケンス
        中断をまとめて行う)。移動系のenable_button(デッドマン)とは独立に扱う
        (安全機能のため、デッドマンを離していても常に効くようにする)。"""
        if self.estop_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.estop_button_ < len(buttons)
                   and bool(buttons[self.estop_button_]))
        if pressed and not self._prev_estop_button_pressed_:
            self._call_trigger(self._estop_client_, '緊急停止(emergency_stop)')
        self._prev_estop_button_pressed_ = pressed

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
        self._update_estop_button(msg)
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
        vel_names = []
        vel_values = []

        # レート方式。入力が中立/デッドマン未押下の間は目標を現在値に同期する
        # だけでpublishに含めない(離した位置を保持する。autoモードの動作を
        # 妨げず、モード切替時の急変も防ぐ)。
        if self.velocity_mode_enabled_:
            # joy速度指令モード(2026-09-09追加、declare_parameter部コメント参照)。
            # target_z_を積分せず、スティック入力をそのまま速度[m/s]として毎周期
            # 送る(離せば0を送り続けて実機側の速度PIDでブレーキがかかる、
            # trajectory_follower_node.py _velocity_mode_target_rpm参照)。
            # 位置モードへ戻したときに違和感なく再開できるよう、target_z_は
            # 引き続き実位置に同期しておく。
            vel_names.append('z_joint')
            vel_values.append(z_in * self.z_speed_ if enabled else 0.0)
            if self.has_current_state_:
                self.target_z_ = self._current_z_
        elif enabled and z_in != 0.0:
            self.target_z_ = clamp(self.target_z_ + z_in * self.z_speed_ * self.dt_, Z_LOWER, Z_UPPER)
            names.append('z_joint')
            positions.append(self.target_z_)
        elif self.has_current_state_:
            self.target_z_ = self._current_z_

        if self._xy_move_mode_:
            # SHAREボタンでトグル(既定OFF、_update_xy_move_toggle参照)の間は、
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
            # theta_jog_enabled_が既定falseの間、root_theta_jointはjoyから直接
            # ジョグしない(declare_parameter部コメント参照。command_gui_nodeの
            # 既存の回収/投入シーケンスによる自動位置合わせに任せる)。current
            # 状態への同期だけは続け、joyからの手動制御が無効の間もtarget_theta_が
            # 古い値のまま固定されないようにする。
            if self.theta_jog_enabled_ and enabled and theta_in != 0.0:
                self.target_theta_ = clamp(
                    self.target_theta_ + theta_in * self.theta_speed_ * self.dt_,
                    ROOT_THETA_LOWER, ROOT_THETA_UPPER)
                names.append('root_theta_joint')
                positions.append(self.target_theta_)
            elif self.has_current_state_:
                self.target_theta_ = self._current_theta_

            if self.velocity_mode_enabled_:
                # z_jointと同じ理由(velocity_mode_enabled_の分岐参照)。XY移動
                # モード中はr_jointがXY変換側で扱われるため、ここ(関節モード)
                # でのみ速度指令化する。
                vel_names.append('r_joint')
                vel_values.append(r_in * self.r_speed_ if enabled else 0.0)
                if self.has_current_state_:
                    self.target_r_ = self._current_r_
            elif enabled and r_in != 0.0:
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

        if vel_names:
            vel_out = JointState()
            vel_out.header.stamp = self.get_clock().now().to_msg()
            vel_out.name = vel_names
            vel_out.velocity = vel_values
            self.vel_pub_.publish(vel_out)

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
