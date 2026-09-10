#!/usr/bin/env python3
"""
soki_sim: joyパッケージのjoy_node(/joy, sensor_msgs/Joy)を購読し、スティック/十字キー
入力を関節空間で直接ジョグしてtrajectory_follower_nodeの/joint_targetsへpublishする
手動操作ノード。

操作割り当て(デフォルト。実際のコントローラのaxes/buttons番号は`ros2 topic echo /joy`で
確認し、axis_theta_l2/axis_theta_r2/axis_z/axis_r・button_tip_theta_l1/
button_tip_theta_r1・invert_*・pump_toggle_buttonパラメータで合わせること):
  右スティック上下 -> z_joint (axis_z, デフォルト4。2026-09-03、ユーザー指定:
                                         「Zは右スティック上下で」により変更
                                         (以前は左スティック上下)。常にこの軸が
                                         z_jointを操作する)
  L1/R1ボタン      -> tip_theta_joint  (button_tip_theta_l1/button_tip_theta_r1、
                                         デフォルト4/5。手先θ、可動域は電源投入
                                         位置から±135degの制限は2026-09-10に廃止
                                         2026-09-10、ユーザー
                                         指定:「手先θの手動ジョグはL1、R1で行う」
                                         により右スティック左右(axis_tip_theta)
                                         から変更。R1側を正方向として押下状態の
                                         差(digital)を入力値にする、root_thetaの
                                         L2/R2トリガー方式と同じ考え方だがこちらは
                                         アナログトリガーではなく通常ボタン)
  L2/R2トリガー    -> root_theta_joint(axis_theta_l2/axis_theta_r2、デフォルト2/5。
                                         2026-09-09、当初はmanualブランチの操作
                                         方針「根本θのみ自動で位置合わせ、RとZは
                                         人が速度制御」によりroot_thetaの直接
                                         ジョグを既定無効(theta_jog_enabled=
                                         false)にし、command_gui_nodeの既存の
                                         回収/投入シーケンスによる自動位置合わせに
                                         任せていたが、同日中にユーザー指定:
                                         「手動でも根本θを操作できるように」
                                         「L2、R2で根本θを操作」により既定を
                                         theta_jog_enabled=true・操作方法を左
                                         スティック左右からL2/R2トリガーへ変更
                                         した(自動シーケンスとの併用は引き続き
                                         可能。R2側を正方向として押下量の差を
                                         入力値にする、_trigger_pressed参照)。
                                         theta_jog_enabled=falseで従来通り自動
                                         位置合わせのみに戻せる)。
  左スティック上下 -> r_joint          (axis_r、デフォルト1。2026-09-03、
                                         ユーザー指定:「XYは左スティック」により
                                         axis_yを4→1に変更した名残。axis_rも右
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
  PSボタン         -> ソフト緊急停止トグル (estop_button, デフォルト10。PS4/PS5
                                         コントローラの一般的なLinuxドライバ
                                         割り当てを仮定した値、実機で要確認。
                                         2026-09-08追加、2026-09-09トグル化。
                                         立ち上がりエッジ即時で、現在estop_active
                                         でなければcommand_gui_nodeの
                                         /emergency_stopを、既にestop_active中
                                         なら/clear_emergency_stopを呼ぶ(GUI本体の
                                         「緊急停止」「解除」ボタンと同じ効果。
                                         trajectory_follower_nodeのcubemars/robomas
                                         出力凍結・GUI自動シーケンス中断/解除を行う。
                                         GUI本体の「緊急停止」ボタン自体は誤操作
                                         防止のためトグルにしていないが、PSボタンは
                                         コントローラを手放さず片手で即座に再始動
                                         できる操作性を優先しトグルにする)。移動系の
                                         enable_button(デッドマン)とは独立に扱う
                                         (安全機能のため常に効くようにする))
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
                                         tip_theta_joint = TIP_THETA_FOLLOW_SIGN*root_theta_jointを
                                         毎周期指令し続け(回収シーケンスと同じ
                                         追従式)、右スティック左右による手動ジョグは
                                         無視する。OFFにすると従来通り右スティック
                                         左右で独立にジョグできる。既定でON。
                                         command_gui_nodeのset_tip_theta_follow_theta
                                         サービス(std_srvs/SetBool)経由でも切替可能で、
                                         投入(L4/R4)シーケンス開始時にGUI側が自動で
                                         OFFにする(投入シーケンスは手先θを固定値
                                         shoot_tip_theta_radへ制御するため、追従ONの
                                         ままだと本ノードが毎周期TIP_THETA_FOLLOW_SIGN*root_thetaへ上書き
                                         して競合するのを防ぐ、_on_set_tip_theta_
                                         follow_srv参照))
  SHAREボタン      -> 低速モードトグル (low_speed_toggle_button, デフォルト8。
                                         PS4/PS5コントローラの一般的なLinux
                                         ドライバ割り当てを仮定した値、実機で要確認。
                                         2026-09-09追加、ユーザー指定:「SHAREボタンで
                                         低速モードと通常モードを切り替え」。以前は
                                         ワールドXYジョグへの切り替え(XY移動モード)に
                                         割り当てていたが、joy速度指令モード中心の
                                         運用ではワールドXYジョグの使い道が無くなった
                                         ため廃止し(ユーザー指摘:「SHAREにはXYモード
                                         があったと思うが不要」)、低速モードの
                                         トグルに差し替えた。ONの間、theta_speed/
                                         z_speed/r_speed/tip_theta_speedをすべて
                                         low_speed_multiplier(既定0.3、command_gui_
                                         nodeの「joy速度」パネルから調整可能)倍に
                                         落として精密操作しやすくする)

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
import time

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

# tip_theta(手先θ)の可動域について。
# 2026-09-10、機構側に物理リミット(当てて止めるストッパ)が付いたのに伴い、
# それまでの固定±135deg制限は廃止した(ユーザー指定:「手先θにリミットをつけた、
# 物理的に当てて止めるものなので電流値を見て入力を止められないか」「この角度
# 制限はなくしていい」)。代わりにtrajectory_follower_nodeが実電流+実速度から
# 機械端を検出して指令を止める(TIP_THETA_STALL_*、_update_tip_theta_stall参照)。
# このノード側では角度でのクランプは行わない。
#
# ただしジョグ・追従の目標値が実機の現在角度より先へ無制限に走るのは別問題なので、
# TIP_THETA_JOG_LEAD_RADで先行量を頭打ちにする。これが無いと、機械端に当たって
# 実機が止まっている間もボタンを押し続けたぶんだけ目標だけが進み続け、
# 離した後もその差を詰めるまで動き続ける(実機が追いつけない場合も同じ)。
TIP_THETA_JOG_LEAD_RAD = math.radians(15.0)

# 手先θ追従の符号: tip_theta_joint = TIP_THETA_FOLLOW_SIGN * root_theta_joint。
# 吸着パッド3個の展開軸をワークの行(ワールドX軸)と平行に保つための関係式。
# 2026-09-10、ユーザー報告「手先追従時はモーターの回転が逆、手動操作時の
# コントローラーとの対応づけはあっている」により -1.0 から +1.0 へ修正した。
# 手動ジョグ(L1/R1)はinvert_tip_theta未設定(sign_tip_theta_=+1)のまま正しい向きに
# 動いているので、関節角の向きの定義自体は合っている。つまり追従の関係式だけが
# 逆だった。原因は2026-09-08のtip_theta駆動系変更(CubeMars AK40-10の直接駆動から
# RoboMas M2006 + タイミングベルト(20T/28T)へ)で、モータが機構のどちら側に付くかが
# 変わり、root_thetaの回転に対して手先が回る向きが反転したため。-1.0はCubeMars
# 時代(2026-09-03)の値をそのまま流用していた。
# joy_teleop_node.pyとcommand_gui_node.pyの両方に同じ値を置くこと(片方だけ直すと
# 手動運転中の追従と回収シーケンスで向きが食い違う)。
TIP_THETA_FOLLOW_SIGN = 1.0

# trajectory_follower_node.py/display.launch.pyのINITIAL_ROOT_THETA_RAD/zerosと
# 一致させること(sim起動直後、フィールドに平行・ハンドが右側になる向き、
# 2026-09-03)。
INITIAL_ROOT_THETA_RAD = -math.pi / 2.0


# ros2 param setで実行中に差し替えられるようにするボタン/軸のindexパラメータ
# (パラメータ名 -> コンストラクタでキャッシュしている属性名、_on_set_parameters参照)。
# 新しくボタンを追加したらここにも登録すること。
_BUTTON_INDEX_PARAMS = {
    'enable_button': 'enable_button_',
    'pump_toggle_button': 'pump_toggle_button_',
    'pickup_confirm_button': 'pickup_confirm_button_',
    'estop_button': 'estop_button_',
    'shoot_start_l4_button': 'shoot_start_l4_button_',
    'shoot_start_r4_button': 'shoot_start_r4_button_',
    'tip_theta_follow_theta_button': 'tip_theta_follow_theta_button_',
    'low_speed_toggle_button': 'low_speed_toggle_button_',
    'button_tip_theta_l1': 'button_tip_theta_l1_',
    'button_tip_theta_r1': 'button_tip_theta_r1_',
}


# タイマー遅れ警告のしきい値と間引き間隔(_check_timer_slip参照)。
_TIMER_SLIP_WARN_RATIO = 2.0        # 実測dtが公称周期のこの倍を超えたら遅れとみなす
_TIMER_SLIP_LOG_INTERVAL_SEC = 5.0  # まとめて警告を出す間隔


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        # root_theta_jointの手動ジョグ入力軸(2026-09-09、L2/R2トリガーへ変更。
        # 以前は左スティック左右(axis_theta、axes[0])だったが、ユーザー指定:
        # 「L2、R2で根本θを操作」。PS4/PS5コントローラの標準的なLinux joy_node
        # マッピング(axes[2]=L2, axes[5]=R2、他の軸(axis_z=4/axis_r=1/
        # axis_select_col・row=6・7)もこのマッピング前提の値に
        # なっている)を仮定した値、実機で要確認。L2/R2は多くのドライバで未使用時
        # +1.0・全押しで-1.0を返す(初回押下までは0.0のままの既知の癖がある
        # ドライバもある)。theta_trigger_rest_value参照。
        self.declare_parameter('axis_theta_l2', 2)
        self.declare_parameter('axis_theta_r2', 5)
        # L2/R2軸の「未使用時」の値(_trigger_pressed参照)。標準的なPS4/PS5
        # コントローラのLinuxドライバでは1.0(全押しで-1.0)。ドライバによって
        # 0.0(全押しで-1.0や1.0)等になる場合はここを実機に合わせて変更すること。
        self.declare_parameter('theta_trigger_rest_value', 1.0)
        # z_jointの手動ジョグ軸(右スティック上下、2026-09-03、同日ユーザー
        # 指定:「Zは右スティック上下で」により左スティック上下から変更。
        # XY移動モード・関節モードいずれでも常にこの軸がz_jointを操作する)。
        self.declare_parameter('axis_z', 4)
        # r_jointの手動ジョグ軸(左スティック上下、2026-09-03、同日z_jointが
        # 右スティック上下(axis_z=4)へ移ったため、そのままだと関節モードで
        # r_jointとz_jointが同時に動いてしまう。競合を避けるため十字キー上下
        # →右スティック上下→左スティック上下、と再度変更した)。
        self.declare_parameter('axis_r', 1)
        # 手先θ(tip_theta_joint)の手動ジョグボタン(2026-09-03追加、2026-09-10
        # 右スティック左右(axis_tip_theta)からL1/R1ボタンへ変更、ユーザー指定:
        # 「手先θの手動ジョグはL1、R1で行う」)。標準的なPS4/PS5コントローラの
        # 一般的なLinuxドライバ割り当て(4=L1、5=R1)を仮定した値、実機で要確認。
        # -1で該当ボタン無効。R1側を正方向として押下状態の差(digital)を入力値に
        # する(_timer_callback参照)。
        self.declare_parameter('button_tip_theta_l1', 4)
        self.declare_parameter('button_tip_theta_r1', 5)
        # 既定true(2026-09-09、L2/R2トリガーへの操作方法変更後、ユーザー報告:
        # 「左右を反転」により回転方向を反転)。
        self.declare_parameter('invert_theta', True)
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
        # 低速モード(SHAREボタン、2026-09-09追加。以前はXY移動モードのトグルに
        # 割り当てていたが、joy速度指令モード中心の運用ではワールドXYジョグの
        # 使い道が無くなったため、SHAREボタンごと低速モードのトグルに差し替えた。
        # ユーザー指摘:「SHAREにはXYモードがあったと思うが不要」)。ONの間、
        # theta_speed/z_speed/r_speed/tip_theta_speedをlow_speed_multiplier倍に
        # 落として精密操作しやすくする(_timer_callback参照)。
        self.declare_parameter('low_speed_multiplier', 0.3)
        # z/rのjoy出力を位置目標(target_z_/target_r_を積分してjoint_targetsへ)
        # ではなく、trajectory_follower_nodeの速度モード(robomas_velocity_mode)
        # 向けにスティック入力をそのまま速度指令(joint_velocity_targets)として
        # 送るモード(2026-09-09追加、command_gui_nodeの「joy速度指令モード」
        # チェックボックスから切り替える想定)。ONの間、z_speed_/
        # r_speed_はそのまま「フル入力時の速度[m/s]」としてtarget_z_/target_r_の
        # 積分ではなく直接の速度指令値に使う(既存のレート方式と単位を揃えるため
        # 新規パラメータは追加しない)。既定true(2026-09-09、手動移動にフォーカス
        # するmanualブランチでの方針変更によりデフォルトの移動モードを速度指令へ。
        # trajectory_follower_node側のrobomas_velocity_modeと揃えること)。
        self.declare_parameter('velocity_mode_enabled', True)
        # root_theta_jointの手動joyジョグ(L2/R2、axis_theta_l2/axis_theta_r2)を
        # 許可するか。2026-09-09当初はmanualブランチの操作方針(「根本θのみ自動で
        # 位置合わせ、RとZは人が速度制御で操作」。root_thetaはcommand_gui_nodeの
        # 既存の回収/投入シーケンスによる自動位置合わせに任せ、joyからは直接
        # 動かさない)により既定falseにしていたが、同日中にユーザー指定:「手動でも
        # 根本θを操作できるように」により既定trueへ変更。自動シーケンスとの併用も
        # 引き続き可能(シーケンス側のtheta移動とジョグ入力が同時に来た場合は
        # 単純に後着優先、target_theta_を両方が共有して書き換えるため)。
        self.declare_parameter('theta_jog_enabled', True)
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
        # 低速モードトグルボタン(SHAREボタンを想定、2026-09-09追加。以前はXY
        # 移動モードのトグルだった、declare_parameter('low_speed_multiplier'...)
        # のコメント参照)。-1ならボタン操作無効。
        self.declare_parameter('low_speed_toggle_button', 8)
        # GUI上のワーク選択カーソル移動軸(十字キー、2026-09-03追加)。多くの
        # Linuxジョイスティックドライバでは十字キーがaxes配列の末尾2要素として
        # 出てくる想定。-1で該当方向を無効。
        self.declare_parameter('axis_select_col', 6)   # 十字キー左右
        self.declare_parameter('axis_select_row', 7)   # 十字キー上下
        self.declare_parameter('invert_select_col', False)
        self.declare_parameter('invert_select_row', False)

        self.axis_theta_l2_ = int(self.get_parameter('axis_theta_l2').value)
        self.axis_theta_r2_ = int(self.get_parameter('axis_theta_r2').value)
        self.theta_trigger_rest_value_ = float(self.get_parameter('theta_trigger_rest_value').value)
        # L2/R2の「初回押下までraw=0.0のままの既知の癖があるドライバもある」問題
        # (theta_trigger_rest_value宣言部のコメント参照)への対策。未較正の軸で
        # raw==0.0が来た場合、それが「本当に半押し(rest未満)」なのか「まだ一度も
        # 触れていないため未初期化のまま0.0が来ている」のか区別できない。前者だと
        # 解釈すると(rest_value=1.0のとき)押下量0.5相当の値をL2/R2の一方だけが
        # 実際には触れていないのに出し続け、theta_inがゼロにならず根本θが勝手に
        # 動き続ける(2026-09-10、ユーザー報告:「ソフト緊急停止解除直後に根本θを
        # PSコンで操作すると入力が入りっぱなしになる」。estopトグル操作の後、
        # セッション中まだ一度も触れていない側のL2/R2がこの状態になっていたと
        # 考えられる)。そのため軸ごとにraw!=0.0を一度でも観測するまでは未較正
        # 扱いとし、未較正の間raw==0.0は「未押下(0.0)」とみなす(_trigger_pressed
        # 参照。rest_value=0.0の構成では元々(rest-raw)/2=0になるため影響しない)。
        self._trigger_calibrated_ = {self.axis_theta_l2_: False, self.axis_theta_r2_: False}
        self.axis_z_ = int(self.get_parameter('axis_z').value)
        self.axis_r_ = int(self.get_parameter('axis_r').value)
        self.button_tip_theta_l1_ = int(self.get_parameter('button_tip_theta_l1').value)
        self.button_tip_theta_r1_ = int(self.get_parameter('button_tip_theta_r1').value)
        self.axis_select_col_ = int(self.get_parameter('axis_select_col').value)
        self.axis_select_row_ = int(self.get_parameter('axis_select_row').value)
        self.sign_theta_ = -1.0 if self.get_parameter('invert_theta').value else 1.0
        self.sign_z_ = -1.0 if self.get_parameter('invert_z').value else 1.0
        self.sign_r_ = -1.0 if self.get_parameter('invert_r').value else 1.0
        self.sign_tip_theta_ = -1.0 if self.get_parameter('invert_tip_theta').value else 1.0
        self.sign_select_col_ = -1.0 if self.get_parameter('invert_select_col').value else 1.0
        self.sign_select_row_ = -1.0 if self.get_parameter('invert_select_row').value else 1.0
        self.enable_button_ = int(self.get_parameter('enable_button').value)
        self.theta_speed_ = float(self.get_parameter('theta_speed').value)
        self.z_speed_ = float(self.get_parameter('z_speed').value)
        self.r_speed_ = float(self.get_parameter('r_speed').value)
        self.tip_theta_speed_ = float(self.get_parameter('tip_theta_speed').value)
        self.low_speed_multiplier_ = float(self.get_parameter('low_speed_multiplier').value)
        self.velocity_mode_enabled_ = bool(self.get_parameter('velocity_mode_enabled').value)
        self.theta_jog_enabled_ = bool(self.get_parameter('theta_jog_enabled').value)
        self.deadzone_ = float(self.get_parameter('deadzone').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.dt_ = 1.0 / update_rate_hz
        # 直前に_timer_callbackが走ったmonotonic時刻(未実行ならNone)。
        # ジョグの積分に「タイマー周期の公称値(self.dt_)」ではなく実測経過時間を
        # 使うため(2026-09-10追加、ユーザー報告:「手先θの速度のレンジが全然
        # ちがうときがある。手動操作で」)。ROS2のタイマーはCPU負荷で遅れる
        # (rviz・command_gui_node・ros2can GUIを同時に動かす本番構成では特に。
        # 2026-09-10にros2canを--noguiからGUIありへ戻したぶん負荷が増えている)。
        # 公称値で積分すると、タイマーが遅れたぶんだけ目標の進みが実時間に対して
        # 遅くなり、同じボタンを同じだけ押しても速度が変わって見える。
        self._last_timer_monotonic_ = None
        # タイマー遅れの監視(_timer_callback参照)。実測dtが公称周期のこの倍率を
        # 超えたら「タイマーが遅れている」とみなし、_TIMER_SLIP_LOG_INTERVAL_SEC
        # ごとにその区間の最悪値をまとめて1行警告する。毎回出すとログが流れて
        # 使い物にならないため間引く。積分自体は実測dtで補正済みなので、この
        # 警告は「なぜジョグの体感速度が変わるのか」を切り分けるための情報。
        self._timer_slip_worst_dt_ = 0.0
        self._timer_slip_count_ = 0
        self._timer_slip_last_log_ = None
        self.pump_toggle_button_ = int(self.get_parameter('pump_toggle_button').value)
        self.pickup_confirm_button_ = int(self.get_parameter('pickup_confirm_button').value)
        self.estop_button_ = int(self.get_parameter('estop_button').value)
        self.shoot_start_l4_button_ = int(self.get_parameter('shoot_start_l4_button').value)
        self.shoot_start_r4_button_ = int(self.get_parameter('shoot_start_r4_button').value)
        self.tip_theta_follow_theta_button_ = int(
            self.get_parameter('tip_theta_follow_theta_button').value)
        self.low_speed_toggle_button_ = int(self.get_parameter('low_speed_toggle_button').value)

        # 現在の目標関節角度(スティック/十字キー入力をここへ積分していく)。
        # /mixed_joint_statesを受信するまでは、trajectory_follower_node起動直後の
        # 実際の静止姿勢に合わせておく。ここを可動域の中央など実際と異なる値に
        # すると、/mixed_joint_states受信前に一部の軸だけ操作した場合でも同じ
        # JointStateメッセージに含まれる他の軸がその適当な初期値へ動いてしまう。
        self.target_theta_ = INITIAL_ROOT_THETA_RAD
        self.target_z_ = 0.0
        self.target_r_ = 0.0
        self.target_tip_theta_ = 0.0
        # 関節ごとの「実値を一度でも受け取ったか」。real_joint_bridge_nodeは
        # CubeMars群(root_theta)とROBOMAS群(z/r/tip_theta)を別メッセージで
        # publishするため、1メッセージに全関節が揃うとは限らない
        # (_on_mixed_joint_state参照)。has_current_state_は従来どおり
        # 「root_theta/z/rの3つとも一度は受け取った」を意味する。
        self._has_root_theta_state_ = False
        self._has_z_state_ = False
        self._has_r_state_ = False
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

        # ソフト緊急停止ボタン(PSボタン、2026-09-08追加、2026-09-09トグル化)。
        # 立ち上がりエッジ即時でcommand_gui_nodeの/emergency_stopまたは
        # /clear_emergency_stopを呼ぶ(_update_estop_button参照)。現在の状態は
        # trajectory_follower_nodeがlatchedでpublishするestop_activeを購読して
        # 判定する(GUIハンドパネル等の他経路での解除とも状態がズレないように
        # するため、hand_pump_state購読と同じ考え方)。
        self._estop_active_ = False
        estop_active_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'estop_active', self._on_estop_active, estop_active_qos)
        self._prev_estop_button_pressed_ = False
        self._estop_client_ = self.create_client(Trigger, 'emergency_stop')
        self._estop_clear_client_ = self.create_client(Trigger, 'clear_emergency_stop')

        # L4/R4へ移動ボタン(□/○ボタン、2026-09-03追加)。command_gui_node側の
        # /shoot_sequence_start_l4・_r4サービスを呼び、固定のシューティングエリア
        # (L4/R4)へ向かわせる。
        self._prev_shoot_start_l4_pressed_ = False
        self._shoot_start_l4_client_ = self.create_client(Trigger, 'shoot_sequence_start_l4')
        self._prev_shoot_start_r4_pressed_ = False
        self._shoot_start_r4_client_ = self.create_client(Trigger, 'shoot_sequence_start_r4')

        # 手先θのroot_theta追従トグル(OPTIONSボタン、2026-09-03追加)。既定でON
        # (ユーザー指定:「デフォルトを自動シーケンスで実行に」)。ONの間は
        # _timer_callbackがtip_theta_joint = TIP_THETA_FOLLOW_SIGN*root_theta_jointを毎周期指令し、
        # 右スティック左右の手動ジョグ入力は無視する(_update_tip_theta_follow_
        # toggle参照)。
        self._tip_theta_follow_theta_ = True
        self._prev_follow_toggle_pressed_ = False
        # OPTIONSボタンで追従をONからOFFへ落とした直後に一度だけTrueになる
        # (_update_tip_theta_follow_toggle -> _timer_callback、2026-09-10追加)。
        # 追従OFF・L1/R1も押していない状態はどこにもpublishしない分岐なので、
        # これが無いとtrajectory_follower_node側のtarget_[tip_theta_joint]が
        # 追従ONだった頃の最後の目標のまま残り、OFFにしても手先θがその目標へ
        # 向かって動き続ける(ユーザー報告:「OPTION押しても追従し続ける」)。
        # tip_theta_jointのmax_velocityは既定0.1rad/sと遅く、135degぶんの移動に
        # 20秒以上かかるため、「ボタンが効いていない」ようにしか見えなかった。
        self._tip_theta_follow_just_released_ = False
        # command_gui_node側からも明示的にON/OFFできるサービス(2026-09-03追加、
        # ユーザー指定:「手先θ追従はシューティングボックスへの自動移動時には
        # 自動で無効化」)。投入(L4/R4)シーケンス開始時にGUI側がこれを呼んで
        # OFFにする(_on_set_tip_theta_follow_srv参照。投入シーケンスは手先θを
        # 固定値shoot_tip_theta_radへ制御するため、追従ONのままだと本ノードが
        # 毎周期TIP_THETA_FOLLOW_SIGN*root_thetaへ上書きして競合するのを防ぐ)。
        self.create_service(
            SetBool, 'set_tip_theta_follow_theta', self._on_set_tip_theta_follow_srv)

        # 低速モード(SHAREボタン、2026-09-09追加。以前はXY移動モードのトグルに
        # 割り当てていたが、joy速度指令モード中心の運用ではワールドXYジョグの
        # 使い道が無くなったため廃止し、SHAREボタンごと低速モードのトグルに
        # 差し替えた。ユーザー指摘:「SHAREにはXYモードがあったと思うが不要」)。
        # ONの間はtheta_speed/z_speed/r_speed/tip_theta_speedをlow_speed_
        # multiplier倍に落とす(_timer_callback参照)。z/r速度モードの実際の
        # 上限はtrajectory_follower_node側のmax_velocityクランプ(_slew_velocity)
        # で決まり、joy側のz_speed/r_speedはそのクランプへ常時飽和させる設計の
        # 大きな値になっている(gains.jsonのコメント参照)。そのため、joy側の目標
        # 速度をlow_speed_multiplier倍しても常にクランプ側の値まで頭打ちになり、
        # 実機の速度が変わらない(2026-09-09、ユーザー報告:「低速モードが機能
        # していない」)。低速モードON/OFFをtrajectory_follower_nodeへも伝え、
        # あちら側のクランプ自体を下げてもらう必要があるため、状態変化のたびに
        # low_speed_activeへlatchedでpublishする(hand_pump_stateと同じ理由の
        # パターン)。
        self._low_speed_enabled_ = False
        self._prev_low_speed_toggle_pressed_ = False
        low_speed_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._low_speed_pub_ = self.create_publisher(Bool, 'low_speed_active', low_speed_qos)
        _low_speed_init_msg = Bool()
        _low_speed_init_msg.data = self._low_speed_enabled_
        self._low_speed_pub_.publish(_low_speed_init_msg)

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
            f'joy_teleop_node started: axis_theta_l2={self.axis_theta_l2_}, '
            f'axis_theta_r2={self.axis_theta_r2_}, axis_z={self.axis_z_}, '
            f'axis_r={self.axis_r_}, '
            f'button_tip_theta_l1={self.button_tip_theta_l1_}, '
            f'button_tip_theta_r1={self.button_tip_theta_r1_}, '
            f'enable_button={self.enable_button_}, '
            f'theta_speed={self.theta_speed_}rad/s, z_speed={self.z_speed_}m/s, '
            f'r_speed={self.r_speed_}m/s, tip_theta_speed={self.tip_theta_speed_}rad/s, '
            f'tip_theta_follow_theta_button={self.tip_theta_follow_theta_button_} '
            f'(follow={self._tip_theta_follow_theta_}), '
            f'low_speed_toggle_button={self.low_speed_toggle_button_}, '
            f'low_speed_multiplier={self.low_speed_multiplier_}, '
            f'axis_select_col={self.axis_select_col_}, axis_select_row={self.axis_select_row_}, '
            f'pickup_confirm_button={self.pickup_confirm_button_}, '
            f'estop_button={self.estop_button_}, '
            f'velocity_mode_enabled={self.velocity_mode_enabled_}, '
            f'theta_jog_enabled={self.theta_jog_enabled_}, '
            f'rate={update_rate_hz}Hz')

    def _on_set_parameters(self, params):
        for p in params:
            if (p.name in ('theta_speed', 'z_speed', 'r_speed', 'tip_theta_speed')
                    and p.value <= 0.0):
                return SetParametersResult(successful=False, reason=f'{p.name} must be positive')
            if p.name == 'low_speed_multiplier' and not (0.0 < p.value <= 1.0):
                return SetParametersResult(
                    successful=False, reason='low_speed_multiplier must be in (0.0, 1.0]')
        for p in params:
            if p.name == 'theta_speed':
                self.theta_speed_ = float(p.value)
            elif p.name == 'z_speed':
                self.z_speed_ = float(p.value)
            elif p.name == 'tip_theta_speed':
                self.tip_theta_speed_ = float(p.value)
            elif p.name == 'r_speed':
                self.r_speed_ = float(p.value)
            elif p.name == 'low_speed_multiplier':
                self.low_speed_multiplier_ = float(p.value)
            elif p.name == 'velocity_mode_enabled':
                self.velocity_mode_enabled_ = bool(p.value)
            elif p.name == 'theta_jog_enabled':
                self.theta_jog_enabled_ = bool(p.value)
            elif p.name in _BUTTON_INDEX_PARAMS:
                # ボタン/軸のindexは実行中でも差し替えられるようにする
                # (2026-09-10追加、ユーザー報告:「オプション押しても追従し続ける」)。
                # これらのindexはどれもPS4/PS5コントローラの一般的なLinuxドライバ
                # 割り当てを仮定した暫定値(declare_parameter部のコメント参照)で、
                # ドライバやコントローラが違うとずれる。以前はコンストラクタで
                # キャッシュしたきり_on_set_parametersが見ていなかったため、
                # ros2 param setしても反映されずノードの再起動が必要だった。
                setattr(self, _BUTTON_INDEX_PARAMS[p.name], int(p.value))
                self.get_logger().info(
                    f'joy_teleop_node: {p.name}を{int(p.value)}に変更しました')
        return SetParametersResult(successful=True)

    def _on_joy(self, msg: Joy):
        self.latest_joy_ = msg

    def _on_pump_state(self, msg: Bool):
        self._pump_on_ = msg.data

    def _on_estop_active(self, msg: Bool):
        self._estop_active_ = msg.data

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
        """PSボタンの立ち上がりエッジで、command_gui_nodeの/emergency_stopまたは
        /clear_emergency_stop(std_srvs/Trigger)を呼ぶ(2026-09-09トグル化、
        ユーザー指定:「緊急停止はPSボタンでトグル」)。現在estop_active_中かどうか
        (購読済み、_on_estop_active参照)で呼び分ける。GUI本体の「緊急停止」/
        「解除」ボタンと同じ効果(trajectory_follower_nodeの出力凍結・ホーミング
        中断・自動シーケンス中断をまとめて行う/解除する)。移動系のenable_button
        (デッドマン)とは独立に扱う(安全機能のため、デッドマンを離していても
        常に効くようにする)。GUI本体の「緊急停止」ボタン自体は誤操作防止のため
        従来通りトグルにしていない(_on_emergency_stop_requestedのdocstring参照)。
        PSボタンはコントローラを手放さず片手で即座に再始動できる必要がある
        操作性を優先し、こちらのみトグルにする。"""
        if self.estop_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.estop_button_ < len(buttons)
                   and bool(buttons[self.estop_button_]))
        if pressed and not self._prev_estop_button_pressed_:
            if self._estop_active_:
                self._call_trigger(self._estop_clear_client_, '緊急停止解除(clear_emergency_stop)')
            else:
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
            if not self._tip_theta_follow_theta_:
                # ONからOFFへ落とした瞬間は、その場で止めるために現在値を1回だけ
                # 目標として送る(_timer_callback側、_tip_theta_follow_just_released_
                # 宣言部のコメント参照)。GUI経由のOFF(_on_set_tip_theta_follow_srv)
                # では立てないこと: あちらは投入シーケンスがshoot_tip_theta_radを
                # 送る直前に呼ぶもので、ここで停止目標を割り込ませるとGUIの目標と
                # 競合する。
                self._tip_theta_follow_just_released_ = True
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
        制御するため、追従ONのままだと本ノードが毎周期TIP_THETA_FOLLOW_SIGN*root_thetaへ上書き
        して競合する)。"""
        self._tip_theta_follow_theta_ = bool(request.data)
        self.get_logger().info(
            'joy_teleop_node: 手先θのroot_theta追従を'
            f'{"ON" if self._tip_theta_follow_theta_ else "OFF"}にしました(GUI経由)')
        response.success = True
        response.message = f'tip_theta_follow_theta={self._tip_theta_follow_theta_}'
        return response

    def _update_low_speed_toggle(self, msg: Joy):
        """SHAREボタンの立ち上がりエッジで、低速モードのローカルなトグル状態を
        反転する(2026-09-09追加、ユーザー指摘:「SHAREボタンで低速モードと通常
        モードを切り替え」)。self._low_speed_enabled_を直接書き換える
        (_timer_callback側の速度計算が参照する)のに加え、trajectory_follower_node
        側のz/r速度モードクランプにも反映させるためlow_speed_activeをpublish
        する(_low_speed_pub_宣言部のコメント参照)。移動系のenable_button
        (デッドマン)とは独立に扱う(ポンプトグルボタンと同じ理由)。"""
        if self.low_speed_toggle_button_ < 0:
            return
        buttons = msg.buttons
        pressed = (0 <= self.low_speed_toggle_button_ < len(buttons)
                   and bool(buttons[self.low_speed_toggle_button_]))
        if pressed and not self._prev_low_speed_toggle_pressed_:
            self._low_speed_enabled_ = not self._low_speed_enabled_
            low_speed_msg = Bool()
            low_speed_msg.data = self._low_speed_enabled_
            self._low_speed_pub_.publish(low_speed_msg)
            self.get_logger().info(
                'joy_teleop_node: 低速モードを'
                f'{"ON" if self._low_speed_enabled_ else "OFF"}にしました')
        self._prev_low_speed_toggle_pressed_ = pressed

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

    def _limit_tip_theta_lead(self, target):
        """手先θの目標値が実角度より先へ走りすぎないよう頭打ちにする
        (TIP_THETA_JOG_LEAD_RAD宣言部のコメント参照)。実角度をまだ受け取って
        いない間は何もしない。"""
        if not self.has_tip_theta_state_:
            return target
        return clamp(target,
                     self._current_tip_theta_ - TIP_THETA_JOG_LEAD_RAD,
                     self._current_tip_theta_ + TIP_THETA_JOG_LEAD_RAD)

    def _on_mixed_joint_state(self, msg: JointState):
        """/mixed_joint_statesから各関節の現在値を取り込む。

        関節ごとに独立して取り込むこと(2026-09-10修正、ユーザー報告:
        「手先θが一切追従しなくなった。初期同じ角度を保持し続けている」)。
        以前はroot_theta/z/rの3つが同じメッセージに揃っていることを前提に
        msg.name.index()を並べ、1つでも欠けるとValueErrorで何も更新せず
        returnしていた。ところがreal_joint_bridge_nodeは実機帰還を
        CubeMars群(name=[root_theta_joint])とROBOMAS群(name=[z_joint,
        r_joint, tip_theta_joint])の別メッセージでpublishするため、実機構成では
        どちらのメッセージも3つ揃わない。その結果、
        ・ROBOMAS群のメッセージが丸ごと捨てられ、z/r/tip_thetaの現在値が
          実機帰還に一切追従しない(_current_tip_theta_/has_tip_theta_state_も
          更新されない)
        ・has_current_state_は、実機帰還が来る前のフォールバック(理想軌道の
          全関節メッセージ、real_joint_bridge_nodeの_on_fallback)が届いた
          ときにしか立たない。そのフォールバック自体、trajectory_follower_node
          のtimer_callbackが has_target_ を待つため最初の/joint_targets受信前は
          publishされない
        という状態だった。
        """
        pos_of = {}
        for name, pos in zip(msg.name, msg.position):
            pos_of[name] = pos

        root_theta = pos_of.get('root_theta_joint')
        if root_theta is not None:
            self._current_theta_ = root_theta
            self._has_root_theta_state_ = True
        z = pos_of.get('z_joint')
        if z is not None:
            self._current_z_ = z
            self._has_z_state_ = True
        r = pos_of.get('r_joint')
        if r is not None:
            self._current_r_ = r
            self._has_r_state_ = True
        self.has_current_state_ = (self._has_root_theta_state_ and self._has_z_state_
                                   and self._has_r_state_)
        # tip_theta_jointはtrajectory_follower_nodeの起動構成次第で無い場合もある
        # ため(root_theta/z/rと違い)、他の3関節とは別に任意扱いにする
        # (2026-09-03追加)。
        tip_theta = pos_of.get('tip_theta_joint')
        if tip_theta is not None:
            self._current_tip_theta_ = tip_theta
            self.has_tip_theta_state_ = True

    def _axis(self, axes, index):
        return axes[index] if 0 <= index < len(axes) else 0.0

    def _button_pressed(self, buttons, index):
        return 0 <= index < len(buttons) and bool(buttons[index])

    def _trigger_pressed(self, axes, index):
        """L2/R2軸(index)の生値を、未使用時0.0・全押しで1.0となる押下量へ変換する
        (theta_trigger_rest_value宣言部のコメント参照)。標準的なPS4/PS5コントローラ
        では未使用時+1.0・全押しで-1.0のため、(rest - raw)/2をクランプする。
        ただしこの軸でまだ一度もraw!=0.0を観測していない(=一度も触れていない
        可能性がある)間は、raw==0.0を(rest-0.0)/2という中途半端な押下量ではなく
        0.0(未押下)として扱う(_trigger_calibrated_宣言部のコメント参照。
        一度でも実際の値が来ればそれ以降は通常通り計算式を信用する)。"""
        raw = self._axis(axes, index)
        if raw != 0.0:
            self._trigger_calibrated_[index] = True
        elif not self._trigger_calibrated_.get(index, True):
            return 0.0
        rest = self.theta_trigger_rest_value_
        return clamp((rest - raw) / 2.0, 0.0, 1.0)

    def _is_enabled(self, msg: Joy):
        if self.enable_button_ < 0:
            return True
        buttons = msg.buttons
        return 0 <= self.enable_button_ < len(buttons) and bool(buttons[self.enable_button_])

    def _check_timer_slip(self, dt, now_monotonic):
        """タイマーの遅れを間引いて警告する(_timer_slip_worst_dt_宣言部参照)。"""
        if dt > self.dt_ * _TIMER_SLIP_WARN_RATIO:
            self._timer_slip_worst_dt_ = max(self._timer_slip_worst_dt_, dt)
            self._timer_slip_count_ += 1
        if self._timer_slip_last_log_ is None:
            self._timer_slip_last_log_ = now_monotonic
            return
        if now_monotonic - self._timer_slip_last_log_ < _TIMER_SLIP_LOG_INTERVAL_SEC:
            return
        self._timer_slip_last_log_ = now_monotonic
        if self._timer_slip_count_ == 0:
            return
        self.get_logger().warning(
            f'joy_teleop_node: 制御周期が遅れています(直近{_TIMER_SLIP_LOG_INTERVAL_SEC:.0f}秒で'
            f'{self._timer_slip_count_}回、最悪{self._timer_slip_worst_dt_ * 1000.0:.0f}ms/'
            f'公称{self.dt_ * 1000.0:.0f}ms)。ジョグの積分は実測時間で補正しているため'
            '目標の進みは狂いませんが、操作の応答が粗くなります。rviz・GUI・ros2can GUIの'
            '同時起動などCPU負荷を減らすか、update_rateを下げてください')
        self._timer_slip_worst_dt_ = 0.0
        self._timer_slip_count_ = 0

    def _timer_callback(self):
        msg = self.latest_joy_
        if msg is None:
            return
        # ジョグ積分用の実測経過時間(_last_timer_monotonic_宣言部のコメント参照)。
        # 初回と、異常に長いギャップ(スリープ・一時停止・重い処理での取りこぼし)は
        # 公称周期の数倍で頭打ちにする。ここを青天井にすると、一瞬詰まっただけで
        # 目標が大きく飛んで実機が急に動く。
        now_monotonic = time.monotonic()
        if self._last_timer_monotonic_ is None:
            dt = self.dt_
        else:
            dt = min(max(now_monotonic - self._last_timer_monotonic_, 0.0), self.dt_ * 5.0)
        self._last_timer_monotonic_ = now_monotonic
        self._check_timer_slip(dt, now_monotonic)
        self._update_pump_toggle(msg)
        self._update_pickup_move(msg)
        self._update_estop_button(msg)
        self._update_shoot_start_l4(msg)
        self._update_shoot_start_r4(msg)
        self._update_tip_theta_follow_toggle(msg)
        self._update_low_speed_toggle(msg)
        self._update_work_selection(msg)
        enabled = self._is_enabled(msg)
        # ソフト緊急停止中(estop_active購読、_on_estop_active参照)は移動系の入力を
        # 一切通さない(2026-09-10追加)。以前はestop状態を見ずにジョグ入力を積分し
        # 続けていたため、estop中にトリガー/スティックを倒していると、
        # (1) target_theta_が実機と無関係に伸び続ける、
        # (2) 速度指令モードのz/rはestop中も非ゼロ速度をpublishし続け、
        #     trajectory_follower_node側の_velocity_targets_に残り続ける、
        # という状態になり、解除した瞬間にその目標/速度がそのまま効いて急に動いた。
        # enabled=Falseにすると各軸とも「現在値へ同期するだけ/速度0を送るだけ」の
        # 分岐に落ちるため、estop解除時点の実機位置から自然に再開できる
        # (trajectory_follower_nodeはestop中も/mixed_joint_statesをpublishし続ける
        # ので、この同期先は脱力して動いた後の実位置になる)。ボタン系(ポンプ・
        # PSボタンのestopトグル等)はデッドマンと同様ここでは止めない。
        if self._estop_active_:
            enabled = False

        # 低速モード(SHAREボタン、2026-09-09追加)。ONの間、以下の全ジョグ速度を
        # low_speed_multiplier倍に落として精密操作しやすくする
        # (_update_low_speed_toggle参照)。
        speed_scale = self.low_speed_multiplier_ if self._low_speed_enabled_ else 1.0

        # root_theta_jointはL2/R2トリガーで操作する(2026-09-09変更、ユーザー指定:
        # 「L2、R2で根本θを操作」)。R2側を正方向として押下量の差を入力値とする
        # (_trigger_pressed参照)。
        theta_in = apply_deadzone(
            self._trigger_pressed(msg.axes, self.axis_theta_r2_)
            - self._trigger_pressed(msg.axes, self.axis_theta_l2_),
            self.deadzone_) * self.sign_theta_
        z_in = apply_deadzone(self._axis(msg.axes, self.axis_z_), self.deadzone_) * self.sign_z_
        r_in = apply_deadzone(self._axis(msg.axes, self.axis_r_), self.deadzone_) * self.sign_r_
        # 手先θ(tip_theta_joint)はL1/R1ボタンでジョグする(2026-09-10、右スティック
        # 左右から変更。declare_parameter部コメント参照)。root_thetaのL2/R2
        # トリガーと同じくR1側を正方向として押下状態の差を入力値にするが、こちらは
        # digitalボタンのため押下量ではなく0.0/1.0の2値の差(-1.0/0.0/1.0)になる。
        tip_theta_in = (float(self._button_pressed(msg.buttons, self.button_tip_theta_r1_))
                        - float(self._button_pressed(msg.buttons, self.button_tip_theta_l1_))
                        ) * self.sign_tip_theta_

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
            vel_values.append(z_in * self.z_speed_ * speed_scale if enabled else 0.0)
            if self.has_current_state_:
                self.target_z_ = self._current_z_
        elif enabled and z_in != 0.0:
            self.target_z_ = clamp(
                self.target_z_ + z_in * self.z_speed_ * speed_scale * dt, Z_LOWER, Z_UPPER)
            names.append('z_joint')
            positions.append(self.target_z_)
        elif self.has_current_state_:
            self.target_z_ = self._current_z_

        # theta_jog_enabled_をfalseにした間だけroot_theta_jointをjoyから直接
        # ジョグしない(declare_parameter部コメント参照。command_gui_nodeの
        # 既存の回収/投入シーケンスによる自動位置合わせのみに任せたい場合用)。
        # current状態への同期だけは続け、joyからの手動制御が無効の間もtarget_
        # theta_が古い値のまま固定されないようにする。
        if self.theta_jog_enabled_ and enabled and theta_in != 0.0:
            self.target_theta_ = clamp(
                self.target_theta_ + theta_in * self.theta_speed_ * speed_scale * dt,
                ROOT_THETA_LOWER, ROOT_THETA_UPPER)
            names.append('root_theta_joint')
            positions.append(self.target_theta_)
        elif self.has_current_state_:
            self.target_theta_ = self._current_theta_

        if self.velocity_mode_enabled_:
            vel_names.append('r_joint')
            vel_values.append(r_in * self.r_speed_ * speed_scale if enabled else 0.0)
            if self.has_current_state_:
                self.target_r_ = self._current_r_
        elif enabled and r_in != 0.0:
            self.target_r_ = clamp(
                self.target_r_ + r_in * self.r_speed_ * speed_scale * dt, R_LOWER, R_UPPER)
            names.append('r_joint')
            positions.append(self.target_r_)
        elif self.has_current_state_:
            self.target_r_ = self._current_r_

        # tip_theta_jointの角度制限は2026-09-10に廃止した(機構側に物理リミットが
        # 付いたため。冒頭の「tip_theta(手先θ)の可動域について」参照)。目標が実角度
        # より先へ走りすぎないようTIP_THETA_JOG_LEAD_RADで頭打ちにするだけにする。
        # trajectory_follower_nodeがtip_theta_joint未構成の場合はpublishしても
        # target_callback側で無視されるだけなので、has_tip_theta_state_の有無に
        # 関わらず常に試みる(2026-09-03追加)。
        # root_theta_joint(names/positions、上記)とは別のメッセージで送る
        # (2026-09-09、ユーザー報告:「手先θの追従をオフにすると動いた」で判明した
        # 不具合の修正)。trajectory_follower_node.target_callbackは同じメッセージに
        # 含まれる関節同士を同時到達させるため、move_time()に基づいてeff_max_vel_
        # 等を自動スケールする(GUIの一括シーケンス移動が狙った挙動)。追従ONの間は
        # 毎周期tip_theta_jointがroot_theta_jointと同じメッセージに同居していたため、
        # L2/R2ジョグ中も常にこの同期スケーリングが働いてしまい、tip_theta_joint側の
        # 実効速度に合わせてroot_theta_joint側の速度まで黙って引き下げられていた
        # (root_theta_jointのmax_velocityを上げても効果が出ない/追従を切ると
        # 速くなる、という形で症状が出ていた)。継続的な追従・ジョグは「複数関節の
        # 一括移動」ではなく関節ごとに独立して動かしたいので、別メッセージにして
        # 同期グループを分離する。
        tip_theta_names = []
        tip_theta_positions = []
        if self._estop_active_:
            # estop中は手先θも指令しない(2026-09-10追加)。追従ON時のこの分岐は
            # enabled(デッドマン)と無関係に毎周期publishするため、上のenabled=False
            # だけでは止まらない。estop中に脱力したroot_thetaが重力で動くと、その
            # 追従先(TIP_THETA_FOLLOW_SIGN*root_theta)を指令し続けてしまい、解除直後に手先θだけが
            # 動き出す。現在値へ同期するだけにして、解除後の値から再開させる。
            if self.has_tip_theta_state_:
                self.target_tip_theta_ = self._current_tip_theta_
        elif self._tip_theta_follow_theta_:
            # OPTIONSボタンでON(既定ON、_update_tip_theta_follow_toggle参照)の
            # 間は、回収シーケンスと同じ追従式(tip_theta=TIP_THETA_FOLLOW_SIGN*root_theta、ワークの
            # 行と平行を保つ)を手動ジョグ中も毎周期指令し続ける。右スティック
            # 左右(tip_theta_in)による独立ジョグはこの間無視する。
            #
            # ただし追従指令を出すのはroot_thetaの実値を一度でも受け取った後に
            # 限る(2026-09-10追加、ユーザー報告:「手先θが起動直後に135度に
            # 向かっている」への対応)。以前は無条件に毎周期publishしていたため、
            # joyメッセージが1つ来た時点で、root_thetaの実値を知る前の初期値
            # (INITIAL_ROOT_THETA_RAD=-90deg)を追従先として tip_theta=+90deg を
            # 指令し始めていた(デッドマンに触れていなくても出る)。さらに
            # root_thetaが可動域の外にいると当時のTIP_THETA_LOWER/UPPERで頭打ちに
            # なり、ちょうど135degへ向かう形になっていた。
            # デッドマン(enabled)は条件に入れないこと: 追従は「root_thetaが
            # どこを向いていてもハンドをワークの行と平行に保つ」ための常時機能で、
            # スティックを離している間も維持する必要がある(2026-09-10、一度
            # enabledを条件に加えたところ「手先θが一切追従しなくなった」と
            # なったため戻した)。
            if self._has_root_theta_state_:
                self.target_tip_theta_ = self._limit_tip_theta_lead(
                    TIP_THETA_FOLLOW_SIGN * self.target_theta_)
                tip_theta_names.append('tip_theta_joint')
                tip_theta_positions.append(self.target_tip_theta_)
            elif self.has_tip_theta_state_:
                self.target_tip_theta_ = self._current_tip_theta_
        elif self._tip_theta_follow_just_released_:
            # OPTIONSで追従をOFFにした直後の1周期だけ、現在値を目標として送って
            # その場で停止させる(_tip_theta_follow_just_released_宣言部参照)。
            # 毎周期publishはしないこと: 追従OFF中も50Hzで送り続けると、GUIの
            # 自動シーケンス(20Hz)が同じ関節へ送る目標を数で押し負けさせてしまう。
            if self.has_tip_theta_state_:
                self.target_tip_theta_ = self._current_tip_theta_
            tip_theta_names.append('tip_theta_joint')
            tip_theta_positions.append(self.target_tip_theta_)
            self._tip_theta_follow_just_released_ = False
        elif enabled and tip_theta_in != 0.0:
            self.target_tip_theta_ = self._limit_tip_theta_lead(
                self.target_tip_theta_
                + tip_theta_in * self.tip_theta_speed_ * speed_scale * dt)
            tip_theta_names.append('tip_theta_joint')
            tip_theta_positions.append(self.target_tip_theta_)
        elif self.has_tip_theta_state_:
            self.target_tip_theta_ = self._current_tip_theta_

        if tip_theta_names:
            tip_theta_out = JointState()
            tip_theta_out.header.stamp = self.get_clock().now().to_msg()
            tip_theta_out.header.frame_id = 'manual'
            tip_theta_out.name = tip_theta_names
            tip_theta_out.position = tip_theta_positions
            self.pub_.publish(tip_theta_out)

        if vel_names:
            vel_out = JointState()
            vel_out.header.stamp = self.get_clock().now().to_msg()
            # joint_targets側(上のout/tip_theta_out)と同じくframe_id='manual'を付ける
            # (2026-09-10修正、ユーザー報告:「R軸自動収納が動かない」)。
            # ここだけ付け忘れていたため、trajectory_follower_node._on_velocity_targets
            # の source = msg.header.frame_id or 'auto' が空文字→'auto'と解釈し、
            # joyの速度指令が「自動シーケンス(command_gui_node)からの指令」として
            # 扱われていた。結果:
            #  - control_modeによる送信元フィルタがjoyの速度指令に対して逆に効く
            #    (control_mode='manual'ではjoyのz/rジョグが弾かれる)。
            #  - スティックを触っていない間も50Hzで送られる速度0.0が'auto'扱いに
            #    なるため、GUIの投入シーケンスのR軸リトラクト(20Hz)を打ち消し、
            #    R軸が微振動するだけでほとんど進まなかった(手動優先の調停も
            #    送信元を区別できず素通りしていた)。
            vel_out.header.frame_id = 'manual'
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
