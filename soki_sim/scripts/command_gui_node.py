#!/usr/bin/env python3
"""
soki_sim: 手先の目標位置(X,Y,Z)を指定して指令を送るGUIノード。

座標系はbase_link原点を基準としたワールド座標(Z軸は鉛直上向き)。
X軸正方向=機体のワーク側を正面としたときの右向き、
Y軸正方向=機体からワークに向かう向き(前方)。
XY平面上でクリックしてピンを置く/数値入力で目標位置を指定し、
逆運動学でroot_theta_joint・z_joint・r_jointに変換して
/joint_targets へpublishする(目標値のみ。瞬時にジャンプさせるのではなく、
trajectory_follower_nodeが台形速度プロファイルで滑らかに追従させたうえで
/mixed_joint_states (joint_state_publisher(_gui)のsource_list) へ出力する。
launch/display.launch.py参照)。
motor_mixer_nodeが/joint_states経由でこれを検知し、motor1/motor2側の値も
自動的に追従計算される。

以下の寸法定数はsoki_sim.urdf.xacroの値と対応させているため、
xacro側を変更した場合はこちらも合わせて変更すること。
  base_height, lift_size_z, arm_length, z_lower/upper, r_lower/upper

複数の目標位置は名前を付けて保存でき、~/.config/soki_sim/points.json に
自動保存される(次回起動時に読み込まれる)。

/mixed_joint_states を購読して現在の関節角度・手先座標(順運動学)をリアルタイム
表示する。また、trajectory_follower_nodeのmax_velocity/max_acceleration/
control_modeパラメータ、joy_teleop_nodeのtheta_speed/z_speed/r_speed(joyジョグの
レート)をGUIから読込・変更できる(rcl_interfacesのget_parameters/set_parameters
サービス経由。対象ノード名ごとにクライアントの組を持ち、request_node_params/
set_node_paramsで共通に扱う。trajectory_follower_node側はmax_velocity/
max_acceleration/control_modeの変更を即座に反映させるためon_set_parameters_
callbackが必要なため、そちらにも対応する変更を入れている)。
control_modeは'auto'(本GUIのみ)/'manual'(joy_teleop_nodeのみ)/'both'(併用)の
3値で、「動作モード」パネルのラジオボタンから即座に切り替えられる。手動操作に
切り替えた後で再び本GUIから送信する際は、意図しないジャンプを避けるため
「現在位置を目標にコピー」ボタンで目標欄を実際の現在位置に合わせてから送信すること。
購読・サービス呼び出しの処理にはspinが必要なため、QTimerによる定期ポーリング
ループの中でrclpy.spin_once(timeout_sec=0)を呼び出す(Qtのイベントループ
(QApplication.exec_())と同じメインスレッドで完結させるシングルスレッド構成。
executorを別スレッドで回す構成も試したが、このROS 2 Jazzy環境ではサービス
クライアントを使った場合にプロセス終了時のrmw層クリーンアップがスレッド競合で
Aborted(core dumped)になることを確認したため採用しなかった。Tkinter版から
PyQt5化した際もこの制約は変わらないため、QTimerもGUIメインスレッドで回す)。

PyQt5が必要(未インストールの場合: sudo apt install python3-pyqt5)。
"""
import functools
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import unicodedata

import yaml

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import (
    QColor, QDoubleValidator, QFont, QFontMetrics, QIcon, QIntValidator, QPainter, QPen, QPixmap,
)
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout,
    QInputDialog, QLabel, QLineEdit, QListWidget, QMessageBox, QPushButton,
    QRadioButton, QScrollArea, QSlider, QTabWidget, QVBoxLayout, QWidget,
)

# soki_sim.urdf.xacro の寸法定数と一致させること
BASE_HEIGHT = 0.06
LIFT_SIZE_Z = 0.08
ARM_LENGTH = 1.244
Z_LOWER, Z_UPPER = 0.0, 0.432
R_LOWER, R_UPPER = -ARM_LENGTH / 2.0, ARM_LENGTH / 2.0

# root_theta: 実機CubeMars(MITモード)の指令可能範囲(±12.5rad、アクチュエータ軸)を
# 外部減速比(112/24)で関節角度に変換した値。soki_sim.urdf.xacroのroot_theta_limitと
# 一致させること。note/hardware_mapping.txt参照(2026-08-27、sim側もこの範囲に制限)。
ROOT_THETA_REDUCTION = 112.0 / 24.0
ROOT_THETA_LIMIT = 12.5 / ROOT_THETA_REDUCTION
ROOT_THETA_LOWER, ROOT_THETA_UPPER = -ROOT_THETA_LIMIT, ROOT_THETA_LIMIT

# tip_theta(手先θ)は機構的にはcontinuous(2026-09-08、CubeMarsからROBOMAS(M2006)へ
# 移行し、CubeMars時代のMIT範囲制約(±511.6°相当)は無くなった。note/hardware_
# mapping.txt「root_theta/tip_thetaはMITモードの都合上そもそも…」参照)だが、
# 関節スライダー(_build_joint_slider_panel)のUI上の目盛り範囲としては手動ジョグ
# 用途で十分な±180degにしておく(実機の可動域自体を制限する値ではない)。
TIP_THETA_LOWER, TIP_THETA_UPPER = -math.pi, math.pi

# lift_link原点(z_joint基準)の地面からの高さオフセット
Z_OFFSET = BASE_HEIGHT + LIFT_SIZE_Z / 2.0
WORLD_Z_LOWER = Z_OFFSET + Z_LOWER
WORLD_Z_UPPER = Z_OFFSET + Z_UPPER
# 手先が原点(旋回軸)から届く最大水平距離(= r_upper + arm_length/2)
MAX_RADIUS = R_UPPER + ARM_LENGTH / 2.0

POINTS_FILE = os.path.expanduser('~/.config/soki_sim/points.json')

# ---- フィールド(ワーク配置環境)・シューティングエリアの寸法定数 ----
# soki_sim.urdf.xacro の該当プロパティと一致させること(ワンクリック移動ボタン用)
FIELD_ROBOT_BOARD_DEPTH = 0.290
FIELD_BOARD_EDGE_OFFSET = FIELD_ROBOT_BOARD_DEPTH / 2.0
WORK_DIAMETER = 0.066
WORK_LENGTH = 0.142
WORK_REST_Z = WORK_DIAMETER / 2.0
FIELD_ROW_GAPS = (0.049, 0.058, 0.058, 0.013)  # 板前端->1行目, 1-2, 2-3, 3-4
FIELD_ROW4_Z_LIFT = 0.020
WORK_COL_PITCH = 0.200

_row_near_x = FIELD_BOARD_EDGE_OFFSET
ROW_X = []
ROW_Z = []
for _i, _gap in enumerate(FIELD_ROW_GAPS):
    _row_near_x += _gap
    ROW_X.append(_row_near_x + WORK_LENGTH / 2.0)
    ROW_Z.append(WORK_REST_Z + (FIELD_ROW4_Z_LIFT if _i == 3 else 0.0))
    _row_near_x += WORK_LENGTH
COL_Y = [(-2.5 + _i) * WORK_COL_PITCH for _i in range(6)]

FIELD_ROBOT_BOARD_CENTER_X = FIELD_BOARD_EDGE_OFFSET - FIELD_ROBOT_BOARD_DEPTH / 2.0

SHOOT_OFFSET_X = 0.305
SHOOT_OFFSET_Y = 0.525
SHOOT_CENTER_X = FIELD_ROBOT_BOARD_CENTER_X - SHOOT_OFFSET_X
SHOOT_BOX_INNER_X = 0.138
SHOOT_BOX_WALL = 0.005
SHOOT_BOX_HEIGHT = 0.145
SHOOT_BOX_OUTER_X = SHOOT_BOX_INNER_X + 2 * SHOOT_BOX_WALL
SHOOT_BOX_GAP = 0.002
SHOOT_BOX_PITCH = SHOOT_BOX_OUTER_X + SHOOT_BOX_GAP
SHOOT_BOX_X = [(-1.5 + _i) * SHOOT_BOX_PITCH for _i in range(4)]
# 手先を箱の上端(開放面)まで下ろす高さ
SHOOT_BOX_Z = SHOOT_BOX_HEIGHT

# ワンクリック移動ボタン用の座標一覧: (ラベル, x, y, z)
# 上のROW_X/COL_Y/SHOOT_*は「奥行き=X, 幅=Y」の中間計算値のため、
# GUIのXY定義(X=右, Y=前方=ワーク方向)に変換してから使う: x = -old_y, y = old_x
WORK_POINTS = [
    [(f'{row + 1}-{col + 1}', -COL_Y[col], ROW_X[row], ROW_Z[row]) for col in range(6)]
    for row in range(4)
]
SHOOT_POINTS = {
    'L': [(f'L{i + 1}', -SHOOT_OFFSET_Y, SHOOT_CENTER_X + SHOOT_BOX_X[i], SHOOT_BOX_Z) for i in range(4)],
    'R': [(f'R{i + 1}', SHOOT_OFFSET_Y, SHOOT_CENTER_X + SHOOT_BOX_X[i], SHOOT_BOX_Z) for i in range(4)],
}
# 実運用で実際に使うシューティングエリアはL4/R4の2箇所のみ(2026-09-03、
# ユーザー指摘: 「シューティングエリアはL4もしくはR4で、ボタンを2つおいておいて」)。
# 「投入エリアへ移動」パネルの固定ボタン2つ(_on_shoot_start_requested)から使う。
SHOOT_FIXED_TARGETS = {'L4': SHOOT_POINTS['L'][3], 'R4': SHOOT_POINTS['R'][3]}

JOINT_NAMES = ['root_theta_joint', 'z_joint', 'r_joint']
# MITゲインパネル(CubeMars)対象関節。JOINT_NAMESとは別物: root_thetaのみが
# CubeMars駆動(z/r/tip_thetaはロボマス駆動、note/hardware_mapping.txt参照。
# tip_thetaは2026-09-08方針変更でCubeMars AK40-10からロボマスM2006(M3)へ移行、
# ロボマスMITゲインパネル(_build_robomas_gain_panel)側で扱う)。
CUBEMARS_JOINT_NAMES = ['root_theta_joint']
# 軌道生成パラメータパネル(max_velocity/max_acceleration)の対象関節。JOINT_NAMES
# (座標指定操作パネルが編集欄を持つ関節)にtip_theta_jointを加えたもの。
# ピック/投入シーケンス(2026-09-03新規)がroot_theta_jointと同時にtip_theta_joint
# も指令するようになったが、tip_theta_jointの速度・加速度は元々GUIから編集する
# 手段が無く、launchファイルの初期値(低速)のまま変更できなかったため、
# 「回収シーケンス中にroot_thetaまで一緒に遅くなる」不具合が発生した(2026-09-03、
# ユーザー報告: trajectory_follower_node.target_callbackの同時到達スケーリングは
# 1メッセージ内の全関節をt_sync=最も時間がかかる関節に合わせるため、
# tip_theta_jointの低いmax_velocity/max_accelerationがroot_theta_jointの実効速度
# まで引きずり下げていた)。
TRAJ_PANEL_JOINT_NAMES = JOINT_NAMES + ['tip_theta_joint']
TRAJ_NODE_NAME = 'trajectory_follower_node'
JOY_NODE_NAME = 'joy_teleop_node'
HOMING_NODE_NAME = 'homing_node'
REAL_JOINT_BRIDGE_NODE_NAME = 'real_joint_bridge_node'
AUTOTUNE_NODE_NAME = 'autotune_node'
# 統合操作タブの「機体ステータス」パネルで起動状況を表示するノード
# (command_gui_nodeがサービス/パラメータ・トピック経由で直接やり取りするノード。
# autotune_nodeは2026-09-09追加)。
STATUS_NODE_NAMES = [TRAJ_NODE_NAME, JOY_NODE_NAME, HOMING_NODE_NAME, REAL_JOINT_BRIDGE_NODE_NAME,
                     AUTOTUNE_NODE_NAME]

# 状態表示灯(赤色LED、2026-09-08追加)の「ノード未起動」判定対象デバイス。
# CubeMars(root_theta/tip_theta、device_id=11)・ROBOMAS(z/r、device_id=21)・
# CAN_HOST自身(device_id=101)のいずれかのserial_rx_{id}_unwrappedが一定時間
# 届いていなければ未起動とみなす(note/note_soki/can_mapping.txt「## 状態表示灯」
# 参照)。device_idはlaunchファイル・note/can_mapping.txtの実機割当と一致させること。
STATUS_DEVICE_IDS = [11, 21, 101]
STATUS_DEVICE_STALE_TIMEOUT_SEC = 2.0

# CAN_HOSTのserial_rx_{device_id}_unwrappedの生値モニタ(配線設定パネルのz/r原点
# センサのノード/スロット割当が実機と合っているか確認する用、2026-09-08追加)。
# device_id自体は「原点センサ・ホーミング配線設定」パネルの入力値(既定101)を
# 都度読む。スロット数はros2can/ros2can/device_profiles.pyのSLOT_COUNTと一致させること。
CAN_HOST_RAW_SLOT_COUNT = 24

# 統合操作タブの「実機セットアップ」パネルから起動する、本番でそのまま使う
# launch構成(note/command.txt「4軸(root_theta/tip_theta/z/r)全軸の実機動作確認。
# 本番でそのまま使う想定」のコマンドと同じ)。real_all_axes_test.launch.pyは
# command_gui_nodeも起動するが、既にこのGUIプロセス自身が動いているため
# launch_gui:=falseでGUIの二重起動を防ぐ。use_viz/use_ros2canは「実機セットアップ」
# パネルのチェックボックスから起動のたびに選べる(2026-09-07追加、ユーザー指摘:
# 「全ノード起動ボタンで起動するrvizとros2canの起動を管理できるチェックボックスを
# 追加」。別途起動済みのrviz/ros2canと二重起動になるのを避けたい場合に使う)。
ALL_AXES_LAUNCH_BASE_CMD = [
    'ros2', 'launch', 'soki_sim', 'real_all_axes_test.launch.py',
    'use_joy:=true', 'launch_gui:=false',
]

# ALL_AXES_LAUNCH_BASE_CMDが起動するノード名(real_all_axes_test.launch.py参照)。
# self._launch_processはこのGUIプロセスのメモリ上の変数でしかないため、GUIを
# 再起動すると前回「全ノード起動」したlaunchプロセスの存在を見失う
# (start_new_session=Trueで独立させているため、GUI終了時にcloseEventで
# 「動かしたまま終了」を選ぶかGUIプロセス自体が異常終了すると道連れにならず
# 生き残り続ける。2026-09-07、実際にこの状態のまま気づかず放置されるインシデントが
# 発生したことへの対策)。GUI起動時に既にこれらのノードが動いていないかを
# _check_existing_launch_nodesで確認する。
ALL_AXES_LAUNCH_NODE_NAMES = {
    'real_joint_bridge_node', 'homing_node', 'autotune_node', 'trajectory_follower_node',
    'hand_node', 'joy_teleop_node',
}

# 機体原点オフセット(soki_sim.urdf.xacroのmachine_origin_x/y/z_joint、base_linkの
# 子であるprismaticジョイント)。trajectory_follower_nodeの管理対象外(滑らか追従は
# 不要な較正値)のため、motor_mixer_nodeと同様/mixed_joint_statesへ直接publishする。
MACHINE_ORIGIN_JOINT_NAMES = ['machine_origin_x_joint', 'machine_origin_y_joint', 'machine_origin_z_joint']
# soki_sim.urdf.xacroのmachine_origin_offset_limitと一致させること
MACHINE_ORIGIN_OFFSET_LIMIT = 1.0

# ハンド取付オフセット(soki_sim.urdf.xacroのhand_offset_x/y/z_joint、tip_linkと
# hand_pitch_linkの間のprismaticジョイント)。machine_origin_*と同じ理由・方式で、
# trajectory_follower_nodeの管理対象外のため/mixed_joint_statesへ直接publishする。
HAND_OFFSET_JOINT_NAMES = ['hand_offset_x_joint', 'hand_offset_y_joint', 'hand_offset_z_joint']
# soki_sim.urdf.xacroのhand_offset_limitと一致させること(2026-09-07、0.1では
# 実機とのズレを補正しきれない場面があったため0.3へ拡大)。
HAND_OFFSET_LIMIT = 0.3

# ---- ピック/投入 自動シーケンス(2026-09-03新規) ----
# 「移動のみGUIで自動化し、吸着ON/シュート実行(ポンプOFF)の判断は人間が行う」という
# 運用イメージに合わせ、根本θ回転前にRを退避させる安全な移動(_safe_move_legs)と、
# 吸着/投入の一時停止(WAIT_HUMAN)を組み合わせたシーケンサ。この退避ロジックは
# ワンクリック移動・XYクリック・保存済みポイント送信等の既存の移動経路には適用しない
# (2026-09-03、ユーザー判断: 新規シーケンスのみに限定)。
SEQ_MOVE_THETA_TOL = 0.02  # rad、到達判定の許容誤差
SEQ_MOVE_LINEAR_TOL = 0.003  # m(z_joint/r_joint共通)、到達判定の許容誤差
SEQ_MOVE_TIMEOUT_SEC = 20.0  # 1レグあたりのタイムアウト(homing_nodeの既定値に合わせる)

# 手先θ(tip_theta_joint)の自動制御(2026-09-03新規、ユーザー指摘: 「ハンドは3つ
# 一気に回収するので手先θはワークの行と平行になるように動く必要がある。シュート時は
# Rと垂直になるように」)。
# 吸着パッド3個の展開軸は、tip_theta_joint=0の姿勢でr方向(アーム伸縮方向)に垂直
# (=hand_indicator_linkの向き、soki_sim.urdf.xacro参照)。root_theta_jointが機体を
# 旋回させると手先ごと同じ角度だけ回るため、パッド展開軸を常にワークの行(GUIの
# ワールドX軸、WORK_POINTSは同一行内でX方向にWORK_COL_PITCH間隔で並ぶ)と平行に
# 保つには、tip_theta_jointをroot_theta_jointと逆方向に同じ量だけ回して打ち消す
# 必要がある(tip_theta_target = -root_theta_target)。_start_pick_sequence参照。
# シュート時は逆に一切打ち消さずtip_theta_joint=一定値(=常にr方向に垂直、
# root_thetaの値によらず幾何学的に成立する)を使うが、この値はまだ実機で検証して
# いない暫定値のため、DEFAULT_SEQUENCE_SETTINGSの'shoot_tip_theta_rad'として
# GUIから調整可能にする(ユーザー指摘: 「シュート時の角度は暫定値である」)。

# GUIのQLineEditへ復元できない場合(gains.json未保存の初回起動時)に使う既定値。
# safe_transit_z_mは安全側(可動範囲上限)に倒しておき、実機確認後に低い値へ調整する想定。
# r_retract_mはradius=0(旋回軸に最も近い位置)。
DEFAULT_SEQUENCE_SETTINGS = {
    'safe_transit_z_m': WORLD_Z_UPPER,
    'r_retract_m': R_LOWER,
    'pickup_z_offset_m': 0.0,
    # 回収シーケンスの自動区間は、ワークに当たる高さ(pickup_z_offset_m基準)まで
    # 一気に降ろさず、その手前(この分だけ高い位置)で一旦止めて人間の「回収実行」
    # 指示を待つ(2026-09-03、ユーザー指摘: 「移動とZをある程度下げる動作は自動、
    # 次にPSコンまたはGUIのボタンで回収を指示、これでワークに当たる高さまで下げる」)。
    'pickup_approach_clearance_m': 0.15,
    # 投入時の手先θ(tip_theta_joint)目標[rad]。r方向(アーム伸縮方向)に垂直となる
    # 値だが、実機で未検証の暫定値(2026-09-03、ユーザー指摘)。
    'shoot_tip_theta_rad': 0.0,
    # パッド収納(/hand_gather_pads)からピッチ投入姿勢への切替(/hand_set_pitch_
    # insert)までの待ち時間[s](2026-09-03、ユーザー指摘: 「収納から姿勢変更
    # までの待機時間も必要。ほぼ同時はまずい」。パッドが物理的に収納し切る前に
    # ピッチが回り始めると干渉する恐れがあるため)。
    'gather_settle_sec': 0.5,
}

# ---- real_joint_bridge.yaml配線設定(初期化用センサID・CubeMars/RoboMasのID・
# 回転方向)----
# これらはreal_joint_bridge_node/homing_nodeが起動時に一度だけ読み込む値で、
# trajectory_follower_nodeのKp/Kd等と違いSetParametersでは実行中ノードに反映
# されない(ノードを再起動してもros2 param setした値は残らず、yamlの値に戻る)。
# GUIの値を実際に使わせるには設定の保存先そのもの、すなわち
# soki_sim/config/real_joint_bridge.yaml(real_joint_bridge_node/homing_nodeの
# 両方が読み込む、コメントで「コードではなくこのファイルを編集すること」と
# 明記されている設定ファイル)を直接書き換える必要がある。
# (key, 表示ラベル, 型) の3つ組。型は 'int'/'float'/'bool'。
# キー名はreal_joint_bridge_node.py/homing_node.pyのdeclare_parameter名と一致させること。
#
# 表示ラベルの用語はnote/can_mapping.txtの記法に合わせて統一する
# (「ID」「番号」「スロット」が場当たり的に混在すると分かりにくいため):
#   ID    = CAN上のデバイス自体を指すCAN_ID(device_id系)
#   番号  = デバイス配下の「ノード」を指す番号(node_index系、0-origin)
#   スロット = ノード(またはデバイス)内の個々のチャンネル/motor位置
#           (local_index系・CubeMars/RoboMasのM{n}位置。note/can_mapping.txtの
#           「ローカルスロット」「M{n}スロット」に対応)
# 2026-08-31方針転換: z_joint/r_jointの位置真値がCAN_HOST外付けENC1/ENC2から
# ROBOMAS内蔵ロータエンコーダのCAN帰還に変わった(real_joint_bridge_node.py/
# homing_node.py参照)ため、ENC1/ENC2配線設定パネルは廃止した。CAN_HOSTは
# z/r原点センサ(SW1/SW2のリミットスイッチ)専用になったため、その設定は
# HOMING_WIRING_FIELDSへ統合した。
# field_specsの形式: [(行見出し または None, [(key, ラベル, 型), ...]), ...]。
# 1タプルが1行に横並びで描画される(ID・ノード・スロット等、意味のある単位を
# 1行にまとめるため。2026-08-31、それまでは単純に2個ずつ機械的に折り返していた
# だけで、関連する項目が別行に分かれてしまっていた)。
ROBOMAS_WIRING_FIELDS = [
    (None, [
        ('robomas_device_id', 'ID', 'int'),
        ('robomas_motor1_index', 'motor1スロット', 'int'),
        ('robomas_motor2_index', 'motor2スロット', 'int'),
        ('robomas_tip_theta_index', 'tip_thetaスロット(M3)', 'int'),
    ]),
    (None, [
        ('motor1_sign', 'motor1回転方向(±1)', 'float'),
        ('motor2_sign', 'motor2回転方向(±1)', 'float'),
        ('tip_theta_sign', 'tip_theta回転方向(±1)', 'float'),
    ]),
]
CUBEMARS_WIRING_FIELDS = [
    (None, [
        ('cubemars_device_id', 'ID', 'int'),
        ('cubemars_root_theta_index', 'root_thetaスロット', 'int'),
    ]),
    (None, [
        ('root_theta_sign', 'root_theta回転方向(±1)', 'float'),
    ]),
]
HOMING_WIRING_FIELDS = [
    (None, [
        ('can_host_device_id', 'CAN_HOST ID', 'int'),
        ('can_host_slots_per_node', 'ノードあたりスロット数', 'int'),
        ('switch_triggered_value', 'SW検出値', 'int'),
    ]),
    ('z原点センサ', [
        ('z_limit_switch_node_index', 'ノード', 'int'),
        ('z_limit_switch_local_index', 'スロット', 'int'),
    ]),
    ('r原点センサ', [
        ('r_limit_switch_node_index', 'ノード', 'int'),
        ('r_limit_switch_local_index', 'スロット', 'int'),
    ]),
    (None, [
        ('robomas_device_id', 'RoboMas ID', 'int'),
        ('robomas_motor1_index', 'motor1スロット', 'int'),
        ('robomas_motor2_index', 'motor2スロット', 'int'),
    ]),
    ('zホーミング回転方向(±1)', [
        ('z_home_motor1_vel_sign', 'motor1', 'float'),
        ('z_home_motor2_vel_sign', 'motor2', 'float'),
    ]),
    ('rホーミング回転方向(±1)', [
        ('r_home_motor1_vel_sign', 'motor1', 'float'),
        ('r_home_motor2_vel_sign', 'motor2', 'float'),
    ]),
    (None, [
        ('z_ref_value_m', 'z原点センサ位置の真値[m]', 'float'),
        ('r_ref_value_m', 'r原点センサ位置の真値[m]', 'float'),
    ]),
]
# z/r上限・下限リミットスイッチ(過走防止の安全停止、trajectory_follower_node、
# 2026-08-31追加)。HOMING_WIRING_FIELDSのz/r原点センサ(下限側、homing_node用)
# とは別パラメータ(同じ配線を指してよいが、ノードが別なので値も別管理)。
LIMIT_SWITCH_WIRING_FIELDS = [
    (None, [
        ('limit_switch_can_host_slots_per_node', 'ノードあたりスロット数', 'int'),
        ('limit_switch_triggered_value', 'SW検出値', 'int'),
    ]),
    ('z下限', [
        ('z_lower_limit_switch_device_id', 'ID', 'int'),
        ('z_lower_limit_switch_node_index', 'ノード', 'int'),
        ('z_lower_limit_switch_local_index', 'スロット', 'int'),
    ]),
    ('z上限', [
        ('z_upper_limit_switch_device_id', 'ID', 'int'),
        ('z_upper_limit_switch_node_index', 'ノード', 'int'),
        ('z_upper_limit_switch_local_index', 'スロット', 'int'),
    ]),
    ('r下限', [
        ('r_lower_limit_switch_device_id', 'ID', 'int'),
        ('r_lower_limit_switch_node_index', 'ノード', 'int'),
        ('r_lower_limit_switch_local_index', 'スロット', 'int'),
    ]),
    ('r上限', [
        ('r_upper_limit_switch_device_id', 'ID', 'int'),
        ('r_upper_limit_switch_node_index', 'ノード', 'int'),
        ('r_upper_limit_switch_local_index', 'スロット', 'int'),
    ]),
]
# ハンド(吸着パッド展開・ワークピッチ変更の2サーボ、ダイヤフラムポンプ)の配線設定
# (hand.yaml、2026-09-03追加)。note/can_mapping.txt「## ハンド」参照。
HAND_WIRING_FIELDS = [
    (None, [
        ('can_slots_per_node', 'ノードあたりスロット数', 'int'),
    ]),
    ('吸着パッド展開サーボ', [
        ('deploy_servo_device_id', 'ID', 'int'),
        ('deploy_servo_node_index', 'ノード', 'int'),
        ('deploy_servo_local_index', 'SERVOスロット', 'int'),
    ]),
    (None, [
        ('deploy_servo_retracted_deg', '収納角度[deg]', 'int'),
        ('deploy_servo_deployed_deg', '展開角度[deg]', 'int'),
    ]),
    (None, [
        ('deploy_servo_offset_deg', '角度オフセット[deg]', 'float'),
    ]),
    ('収納 実機送信角度 手動固定', [
        ('deploy_servo_retracted_can_deg_override', '固定する', 'bool'),
        ('deploy_servo_retracted_can_deg', '角度[deg]', 'float'),
    ]),
    ('展開 実機送信角度 手動固定', [
        ('deploy_servo_deployed_can_deg_override', '固定する', 'bool'),
        ('deploy_servo_deployed_can_deg', '角度[deg]', 'float'),
    ]),
    ('ワークピッチ変更サーボ', [
        ('pitch_servo_device_id', 'ID', 'int'),
        ('pitch_servo_node_index', 'ノード', 'int'),
        ('pitch_servo_local_index', 'SERVOスロット', 'int'),
    ]),
    (None, [
        ('pitch_servo_hold_deg', '保持角度[deg]', 'int'),
        ('pitch_servo_insert_deg', '投入角度[deg]', 'int'),
    ]),
    (None, [
        ('pitch_servo_offset_deg', '角度オフセット[deg]', 'float'),
    ]),
    ('保持 実機送信角度 手動固定', [
        ('pitch_servo_hold_can_deg_override', '固定する', 'bool'),
        ('pitch_servo_hold_can_deg', '角度[deg]', 'float'),
    ]),
    ('投入 実機送信角度 手動固定', [
        ('pitch_servo_insert_can_deg_override', '固定する', 'bool'),
        ('pitch_servo_insert_can_deg', '角度[deg]', 'float'),
    ]),
    ('ダイヤフラムポンプ(MD)', [
        ('pump_device_id', 'ID', 'int'),
        ('pump_node_index', 'ノード', 'int'),
        ('pump_local_index', 'MDスロット', 'int'),
    ]),
    (None, [
        ('pump_duty_percent', 'デューティ[%]', 'float'),
        ('pump_md_pwm_max', 'PWM最大値', 'int'),
    ]),
    ('真空破壊リレー(MDのDIRピンを流用、ポンプON/OFFと連動)', [
        ('vacuum_release_device_id', 'ID', 'int'),
        ('vacuum_release_node_index', 'ノード', 'int'),
        ('vacuum_release_local_index', 'MDスロット', 'int'),
    ]),
    (None, [
        ('vacuum_release_duty_percent', 'デューティ[%](DIRのみ使用、量は無関係)', 'float'),
    ]),
]

# 収納/展開・保持/投入の各角度はsim表示用の論理値(角度制限なし)で、実機へは
# 通常+オフセットを適用して送信するが、対応する*_can_deg_overrideがtrueの間は
# *_can_degをそのまま送信する(hand_node.py _resolve_can_deg参照。いずれの経路も
# クランプは行わない、2026-09-03、ユーザー指摘: 「オフセットをクランプしない。
# 実機との相違があるため手動で固定角度を設定する」)。ハンド配線設定パネルに、
# 実際に送信される角度をその場で確認できるプレビュー表示を追加する。
# (角度キー, オフセットキー, オーバーライド有効キー, オーバーライド角度キー, 表示ラベル)の並び。
HAND_SERVO_PREVIEW_SPECS = [
    ('deploy_servo_retracted_deg', 'deploy_servo_offset_deg',
     'deploy_servo_retracted_can_deg_override', 'deploy_servo_retracted_can_deg', '収納角度 実機送信[deg]'),
    ('deploy_servo_deployed_deg', 'deploy_servo_offset_deg',
     'deploy_servo_deployed_can_deg_override', 'deploy_servo_deployed_can_deg', '展開角度 実機送信[deg]'),
    ('pitch_servo_hold_deg', 'pitch_servo_offset_deg',
     'pitch_servo_hold_can_deg_override', 'pitch_servo_hold_can_deg', '保持角度 実機送信[deg]'),
    ('pitch_servo_insert_deg', 'pitch_servo_offset_deg',
     'pitch_servo_insert_can_deg_override', 'pitch_servo_insert_can_deg', '投入角度 実機送信[deg]'),
]


def _resolve_config_yaml_path(filename):
    """soki_sim/config/{filename}の実ファイルパスを解決する。
    command_gui_node自体がインストール後のパスから実行される(CMakeLists.txtで
    RENAMEインストール)ため、__file__相対ではなくget_package_share_directory経由
    で解決する(_resource_pathと同じ理由)。symlink-installならrealpath()で
    ソースツリー側のファイルが返るため、そちらを直接編集する
    (「コードではなくこのファイルを編集すること」というyaml内コメントの
    運用と一致させる。colcon buildをsymlink-installで行っていない場合は
    次回launchには反映されるがソースツリー側は更新されない)。"""
    try:
        share_dir = get_package_share_directory('soki_sim')
    except PackageNotFoundError:
        return None
    path = os.path.join(share_dir, 'config', filename)
    if not os.path.isfile(path):
        return None
    return os.path.realpath(path)


def _resolve_real_joint_bridge_yaml_path():
    return _resolve_config_yaml_path('real_joint_bridge.yaml')


def _resolve_gains_file_path():
    """軌道生成パラメータ・MIT/robomasゲイン・joy速度のGUI保持値を書き込む
    gains.jsonの実ファイルパスを解決する。実機で有効だった値をホーム
    ディレクトリではなくリポジトリ内(soki_sim/config/gains.json)に置き、
    git管理下でバックアップ・共有できるようにするため、
    _resolve_real_joint_bridge_yaml_pathと同じ方式(get_package_share_directory
    経由、symlink-installならソースツリー側を直接編集)で解決する。パッケージ/
    ファイルが見つからない場合(colcon build未実行の単体起動等)はホーム
    ディレクトリ側にフォールバックする。"""
    try:
        share_dir = get_package_share_directory('soki_sim')
        path = os.path.join(share_dir, 'config', 'gains.json')
        if os.path.isfile(path):
            return os.path.realpath(path)
    except PackageNotFoundError:
        pass
    return os.path.expanduser('~/.config/soki_sim/gains.json')


def _iter_wiring_fields(field_specs):
    """[(行見出し, [(key,label,kind), ...]), ...]形式のfield_specsを
    (key,label,kind)の並びへ平坦化する(読込/保存はグループ行を意識せず
    key単位で処理するため)。"""
    for _row_title, fields in field_specs:
        yield from fields


def _flatten_yaml_node_params(data):
    """{node_name: {ros__parameters: {...}}}形式のyamlを1つのdictにまとめる
    (real_joint_bridge_node/homing_nodeの両方に同名キーがある項目は、正しく
    運用されていれば同じ値のはずなので後勝ちで問題ない)。"""
    flat = {}
    if not isinstance(data, dict):
        return flat
    for node_cfg in data.values():
        if isinstance(node_cfg, dict) and isinstance(node_cfg.get('ros__parameters'), dict):
            flat.update(node_cfg['ros__parameters'])
    return flat


def _format_yaml_int(value) -> str:
    return str(int(value))


def _format_yaml_float(value) -> str:
    s = f'{float(value):.6g}'
    if 'e' in s or 'E' in s:
        s = f'{float(value):.10f}'.rstrip('0')
    if '.' not in s:
        s += '.0'
    return s


def _format_yaml_bool(value) -> str:
    return 'true' if value else 'false'


def _replace_yaml_scalar(text: str, key: str, value_str: str) -> str:
    """text中の`key: value  # comment`形式の行(複数ブロックにまたがる同名キー
    全て)について、value部分だけをvalue_strに置き換える。インデント・コメントは
    完全に保持する(yaml.safe_load+dumpだとコメントが全て失われるため、この
    ファイル特有の1行1パラメータという単純な書式を前提に正規表現で置換する)。"""
    pattern = re.compile(
        r'^([ \t]*' + re.escape(key) + r':[ \t]*)([^\s#]+)', re.MULTILINE)
    return pattern.sub(lambda m: m.group(1) + value_str, text)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def xyz_to_joint(x, y, z):
    """ワールド座標(X,Y,Z) -> (theta, z_joint, r_joint)。可動域外はクランプする。

    X軸正=右向き、Y軸正=機体からワークに向かう前方。
    root_theta_joint角度は旋回軸に対して定義された内部基準(前方=Y+の時にtheta=0)
    に合わせるため、atan2の引数はatan2(-x, y)となる(X/Yをそのまま使うatan2(y,x)ではない)。
    """
    theta_raw = math.atan2(-x, y)
    theta = clamp(theta_raw, ROOT_THETA_LOWER, ROOT_THETA_UPPER)
    radius = math.hypot(x, y)
    raw_r = radius - ARM_LENGTH / 2.0
    raw_z = z - Z_OFFSET
    r = clamp(raw_r, R_LOWER, R_UPPER)
    zj = clamp(raw_z, Z_LOWER, Z_UPPER)
    clamped = (abs(theta_raw - theta) > 1e-9) or (abs(raw_r - r) > 1e-9) or (abs(raw_z - zj) > 1e-9)
    return theta, zj, r, clamped


def _theta_r_from_xy(x, y):
    """xyz_to_jointのtheta/r算出部分のみを取り出したもの(z非依存)。ピック/投入
    シーケンスで、安全高度を保ったまま先にtheta/rを決めるために使う。"""
    theta_raw = math.atan2(-x, y)
    theta = clamp(theta_raw, ROOT_THETA_LOWER, ROOT_THETA_UPPER)
    radius = math.hypot(x, y)
    raw_r = radius - ARM_LENGTH / 2.0
    r = clamp(raw_r, R_LOWER, R_UPPER)
    return theta, r


def joint_to_xyz(theta, zj, r):
    """xyz_to_jointの逆変換(順運動学): (theta, z_joint, r_joint) -> ワールド座標(X,Y,Z)。
    現在の関節角度から手先座標をリアルタイム表示するために使う。"""
    radius = r + ARM_LENGTH / 2.0
    x = -radius * math.sin(theta)
    y = radius * math.cos(theta)
    z = zj + Z_OFFSET
    return x, y, z


def _resource_path(filename):
    """soki_sim/resources/配下のファイルパスを解決する(CMakeLists.txtでインストール
    済み)。パッケージ/ファイルが見つからない場合はNoneを返す。"""
    try:
        share_dir = get_package_share_directory('soki_sim')
    except PackageNotFoundError:
        return None
    path = os.path.join(share_dir, 'resources', filename)
    return path if os.path.isfile(path) else None


def _load_soki_logo_image(target_height: int):
    """soki_sim/resources/soki_logo.pngを読み込み、指定の高さに縮小して返す。
    読めない場合はNoneを返す(ロゴ無しでも動作継続)。"""
    path = _resource_path('soki_logo.png')
    if path is None:
        return None
    pixmap = QPixmap(path)
    if pixmap.isNull():
        return None
    return pixmap.scaledToHeight(target_height, Qt.SmoothTransformation)


def _load_stylesheet():
    """soki_sim/resources/style.qss(モダンダークテーマ)を読み込む。読めない場合は
    空文字を返す(Fusionスタイルのみで動作継続)。"""
    path = _resource_path('style.qss')
    if path is None:
        return ''
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return f.read()
    except OSError:
        return ''


_ROLE_COLORS = {
    'info': '#1a73e8',
    'success': '#1e8e3e',
    'error': '#d93025',
    'muted': '#5f6368',
}


def _set_status(label: QLabel, text: str, role: str = 'muted'):
    """ステータス表示用QLabelのテキストと色をまとめて設定する(Tkinter版のfg=動的
    変更に相当)。"""
    label.setText(text)
    label.setStyleSheet(f'color: {_ROLE_COLORS[role]};')


_ZENKAKU_SIGN_TRANSLATION = str.maketrans({'－': '-', '−': '-', '．': '.'})


def _normalize_numeral_text(text: str) -> str:
    """全角数字・全角マイナス・全角ピリオドを半角へ正規化する。

    日本語IME確定直後は全角文字のままのことがあり、QDoubleValidator/
    QIntValidatorは全角文字をInvalidとして入力自体を弾いてしまう
    (キー入力が反映されず、気づかないまま欄が空/不完全になり適用時に
    「数値を入力してください」エラーになる)。入力の検証前に正規化して
    半角数字として扱えるようにする。"""
    return unicodedata.normalize('NFKC', text).translate(_ZENKAKU_SIGN_TRANSLATION)


class _NormalizingDoubleValidator(QDoubleValidator):
    """全角入力を正規化してから検証するQDoubleValidator。"""

    def validate(self, input_str, pos):
        normalized = _normalize_numeral_text(input_str)
        return super().validate(normalized, min(pos, len(normalized)))


class _NormalizingIntValidator(QIntValidator):
    """全角入力を正規化してから検証するQIntValidator。"""

    def validate(self, input_str, pos):
        normalized = _normalize_numeral_text(input_str)
        return super().validate(normalized, min(pos, len(normalized)))


def make_float_edit(initial: float, width: int = 80) -> QLineEdit:
    """数値入力用QLineEdit(Tkinter版のtk.Entry+DoubleVarに相当)を生成する。"""
    edit = QLineEdit()
    edit.setValidator(_NormalizingDoubleValidator(-1.0e6, 1.0e6, 6))
    edit.setMaximumWidth(width)
    set_float(edit, initial)
    return edit


def get_float(edit: QLineEdit) -> float:
    """QLineEditの内容をfloatとして取得する。数値でない場合はValueErrorを送出する
    (Tkinter版のDoubleVar.get()がtk.TclErrorを送出するのに相当)。"""
    text = _normalize_numeral_text(edit.text().strip())
    if text in ('', '-', '.', '-.'):
        raise ValueError(text)
    return float(text)


def set_float(edit: QLineEdit, value: float):
    edit.setText(f'{value:.6g}')


def make_int_edit(initial: int, width: int = 60) -> QLineEdit:
    """整数入力用QLineEdit(device_id/motor_index等、yamlの型がintのもの用)。"""
    edit = QLineEdit()
    edit.setValidator(_NormalizingIntValidator(-1_000_000, 1_000_000))
    edit.setMaximumWidth(width)
    set_int(edit, initial)
    return edit


def get_int(edit: QLineEdit) -> int:
    text = _normalize_numeral_text(edit.text().strip())
    if text in ('', '-'):
        raise ValueError(text)
    return int(text)


def set_int(edit: QLineEdit, value: int):
    edit.setText(str(int(value)))


class XYPlaneWidget(QWidget):
    """XY平面クリックウィジェット。Tkinter版のtk.Canvas(円クリック・ピン表示・
    現在位置マーカー)をQPainterによる自前描画で置き換えたもの。クリックされた
    ワールド座標をon_clickコールバックへそのまま渡す(可動域クランプは呼び出し側
    (CommandGuiApp._on_canvas_click)の責務)。"""

    def __init__(self, size: int, max_radius: float, margin: int, on_click, parent=None):
        super().__init__(parent)
        self._max_radius = max_radius
        self._margin = margin
        self._on_click = on_click
        self._pin = None
        self._current = None
        self.setFixedSize(size, size)
        self.setStyleSheet('background-color: #f5f5f5; border: 1px solid #555;')

    def set_pin(self, x: float, y: float):
        self._pin = (x, y)
        self.update()

    def set_current(self, x: float, y: float):
        self._current = (x, y)
        self.update()

    def _scale(self) -> float:
        return (self.width() / 2.0 - self._margin) / self._max_radius

    def world_to_widget(self, x: float, y: float):
        c = self.width() / 2.0
        s = self._scale()
        return c + x * s, c - y * s

    def widget_to_world(self, px: float, py: float):
        c = self.width() / 2.0
        s = self._scale()
        return (px - c) / s, (c - py) / s

    def mousePressEvent(self, event):
        x, y = self.widget_to_world(event.pos().x(), event.pos().y())
        self._on_click(x, y)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        c = self.width() / 2.0

        painter.setPen(QColor('#cccccc'))
        painter.drawLine(0, int(c), self.width(), int(c))
        painter.drawLine(int(c), 0, int(c), self.height())

        px_radius = self._scale() * self._max_radius
        painter.setPen(QPen(QColor('#4a90d9'), 1, Qt.DashLine))
        painter.drawEllipse(QPointF(c, c), px_radius, px_radius)

        painter.setPen(QColor('#888888'))
        painter.drawText(int(c) - 44, 14, 'Y+ (ワーク側)')
        painter.drawText(int(c) - 44, self.height() - 6, 'Y- (機体後方)')
        painter.drawText(self.width() - 28, int(c), 'X+')
        painter.drawText(8, int(c), 'X-')
        painter.drawText(int(c) + 10, int(c) + 16, '機体')

        if self._current is not None:
            cx, cy = self.world_to_widget(*self._current)
            painter.setPen(QPen(QColor('#1a7a1a'), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), 5, 5)

        if self._pin is not None:
            cx, cy = self.world_to_widget(*self._pin)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor('red'))
            painter.drawEllipse(QPointF(cx, cy), 5, 5)


class FieldMinimapWidget(QWidget):
    """フィールド(ワーク配置4行6列・シューティングボックスL/R各4)を実寸比率で
    俯瞰する読み取り専用ミニマップ。統合操作タブの「現在状態」パネル用
    (2026-09-07新規、ユーザー要望: 「統合操作画面に簡易的かつ視覚的に機体の
    座標データが分かるような機能」)。XYPlaneWidgetは可動域circle+クリックで
    目標設定する操作用ウィジェットでフィールド要素は描かないため、俯瞰専用に
    別クラスとして分離した。WORK_POINTS/SHOOT_POINTS(_build_field_buttonsが
    ワーク・シューティングボックスのボタン配置に使っているのと同じワールド座標、
    X=右, Y=前方=ワーク方向)をそのまま使うので、フィールド寸法定数を更新すれば
    自動的に追従する。"""

    _PIXELS_PER_METER = 220
    _MARGIN = 14

    def __init__(self, parent=None):
        super().__init__(parent)
        xs = [p[1] for row in WORK_POINTS for p in row] + \
            [p[1] for p in SHOOT_POINTS['L'] + SHOOT_POINTS['R']] + [0.0]
        ys = [p[2] for row in WORK_POINTS for p in row] + \
            [p[2] for p in SHOOT_POINTS['L'] + SHOOT_POINTS['R']] + [0.0]
        self._min_x, self._max_x = min(xs), max(xs)
        self._min_y, self._max_y = min(ys), max(ys)
        width = int(round((self._max_x - self._min_x) * self._PIXELS_PER_METER)) + 2 * self._MARGIN
        height = int(round((self._max_y - self._min_y) * self._PIXELS_PER_METER)) + 2 * self._MARGIN
        self.setFixedSize(width, height)
        self.setStyleSheet('background-color: #f5f5f5; border: 1px solid #555;')
        self._current = None

    def set_current(self, x: float, y: float):
        self._current = (x, y)
        self.update()

    def _to_widget(self, x, y):
        px = (x - self._min_x) * self._PIXELS_PER_METER + self._MARGIN
        # Y+(ワーク方向)を画面の上へ、機体側(Y小さい)を下へ表示する
        # (XYPlaneWidgetのc - y*sと同じ上下反転の考え方)。
        py = self.height() - ((y - self._min_y) * self._PIXELS_PER_METER + self._MARGIN)
        return px, py

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor('#9e9e9e'))
        for row in WORK_POINTS:
            for _label, x, y, _z in row:
                px, py = self._to_widget(x, y)
                painter.drawEllipse(QPointF(px, py), 3, 3)

        for side in ('L', 'R'):
            for label, x, y, _z in SHOOT_POINTS[side]:
                px, py = self._to_widget(x, y)
                # 実運用ではL4/R4のみ使う(SHOOT_FIXED_TARGETS参照)ので、
                # それ以外より枠を太く・塗りつぶして目立たせる。
                is_fixed = label in SHOOT_FIXED_TARGETS
                painter.setPen(QPen(QColor('#e07b00'), 2 if is_fixed else 1))
                painter.setBrush(QColor('#ffcc80') if is_fixed else Qt.NoBrush)
                painter.drawRect(int(px) - 4, int(py) - 4, 8, 8)

        ox, oy = self._to_widget(0.0, 0.0)
        painter.setPen(QPen(QColor('#555555'), 1))
        painter.setBrush(QColor('#cccccc'))
        painter.drawEllipse(QPointF(ox, oy), 6, 6)

        if self._current is not None:
            cx, cy = self._to_widget(*self._current)
            painter.setPen(QPen(QColor('#1a7a1a'), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), 6, 6)


class ZGaugeWidget(QWidget):
    """z_joint由来のワールドZ高さを縦バーで示す読み取り専用ゲージ。統合操作タブの
    「現在状態」パネル用(2026-09-07新規、FieldMinimapWidgetと同じ要望対応。
    XY平面のミニマップだけでは高さが分からないため併設する)。可動範囲は
    WORLD_Z_LOWER/WORLD_Z_UPPER(座標指定操作タブのZ編集欄と同じ範囲)。"""

    def __init__(self, height=200, width=26, parent=None):
        super().__init__(parent)
        self.setFixedSize(width, height)
        self.setStyleSheet('background-color: #f5f5f5; border: 1px solid #555;')
        self._z = None

    def set_z(self, z: float):
        self._z = z
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        if self._z is None:
            return
        span = WORLD_Z_UPPER - WORLD_Z_LOWER
        ratio = clamp((self._z - WORLD_Z_LOWER) / span, 0.0, 1.0) if span > 0 else 0.0
        fill_h = int(round(self.height() * ratio))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor('#4a90d9'))
        painter.drawRect(0, self.height() - fill_h, self.width(), fill_h)
        painter.setPen(QColor('#333333'))
        painter.drawText(2, self.height() - fill_h - 3 if fill_h < self.height() - 12 else 12,
                          f'{self._z:.2f}')


class LedIndicatorWidget(QWidget):
    """状態表示灯(黄色/赤色LED)を模した丸ランプ(2026-09-08新規)。実機の
    CAN_HOST(device_id=101、MULTI1=黄色LED/MULTI2=赤色LED)・status_led.cppと
    同じ「消灯/点灯/点滅(速)/点滅(遅)」の4状態で表現する
    (note/note_soki/can_mapping.txt「## 状態表示灯」参照)。"""

    STATE_OFF = 'off'
    STATE_ON = 'on'
    STATE_BLINK_FAST = 'blink_fast'
    STATE_BLINK_SLOW = 'blink_slow'
    _VALID_STATES = (STATE_OFF, STATE_ON, STATE_BLINK_FAST, STATE_BLINK_SLOW)

    # 実機status_led.cppのCAN_BLINK_INTERVAL_MS(100ms)のトグル間隔感覚に合わせつつ、
    # 画面上で見やすいよう気持ち長め・遅い側とはっきり区別できる値にしてある。
    _TICK_MS = 50
    _BLINK_FAST_PERIOD_MS = 150
    _BLINK_SLOW_PERIOD_MS = 500

    def __init__(self, on_color: str, size=20, parent=None):
        super().__init__(parent)
        self._on_color = QColor(on_color)
        self._off_color = QColor('#3c3c3c')
        self._state = self.STATE_OFF
        self._lit = False
        self._elapsed_ms = 0
        self.setFixedSize(size, size)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(self._TICK_MS)

    def set_state(self, state):
        if state not in self._VALID_STATES:
            state = self.STATE_OFF
        if state != self._state:
            self._state = state
            self._elapsed_ms = 0
            self._apply_lit(state == self.STATE_ON)

    def _on_tick(self):
        if self._state not in (self.STATE_BLINK_FAST, self.STATE_BLINK_SLOW):
            return
        self._elapsed_ms += self._TICK_MS
        period = (self._BLINK_FAST_PERIOD_MS if self._state == self.STATE_BLINK_FAST
                  else self._BLINK_SLOW_PERIOD_MS)
        self._apply_lit((self._elapsed_ms % (period * 2)) < period)

    def _apply_lit(self, lit):
        if lit != self._lit:
            self._lit = lit
            self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QColor('#1a1a1a'))
        painter.setBrush(self._on_color if self._lit else self._off_color)
        painter.drawEllipse(1, 1, self.width() - 2, self.height() - 2)


class CommandGuiNode(Node):

    def __init__(self):
        super().__init__('command_gui_node')
        self.pub_ = self.create_publisher(JointState, 'joint_targets', 10)
        self.mixed_pub_ = self.create_publisher(JointState, 'mixed_joint_states', 10)

        # tip_theta_jointはJOINT_NAMES(手動XY移動が編集欄を持つ関節)には含めないが、
        # ピック/投入シーケンスの到達判定用に現在値を追跡する(2026-09-03追加、
        # _advance_move_step参照)。
        self._current_positions = {
            name: 0.0 for name in
            JOINT_NAMES + ['tip_theta_joint'] + MACHINE_ORIGIN_JOINT_NAMES + HAND_OFFSET_JOINT_NAMES}
        self._current_received = False
        self.create_subscription(JointState, 'mixed_joint_states', self._on_mixed_joint_state, 10)

        # 回収シーケンスが「ワーク手前で自動停止→人間の回収実行指示待ち」の状態に
        # なっているとき、GUIの「回収実行」ボタンだけでなくPSコン(joy_teleop_node)
        # のPSボタンからも進めさせるためのTriggerサービス(2026-09-03追加)。
        # シーケンス状態自体はCommandGuiApp側にあるため、実際の判定・進行処理は
        # set_pick_confirm_handlerで登録されたコールバック(CommandGuiApp._on_pick_
        # confirm_only_requested)に委譲する。2026-09-03、同日当初は「×長押しで
        # 回収実行確定」としていたが、選択中ワークへの「移動」(×の短押し)と
        # 同じボタンを共有していたため、移動が実行中シーケンスを即座に中断・
        # やり直す仕様(下記参照)と衝突し、長押しのつもりで押した瞬間に移動が
        # 先に発火して回収実行待ち状態を壊してしまう不具合が発生した。ユーザー
        # 指定:「回収ボタンをバツ長押しからPSボタンに変更」により、確定は
        # 専用のPSボタンへ分離した(誤操作でワークに接触・吸着してしまうことを
        # 防ぐ安全策として、移動用の×ボタンとは意図的に別ボタンにしている)。
        self._pick_confirm_handler = None
        self.create_service(Trigger, 'pick_sequence_confirm', self._on_pick_sequence_confirm_srv)

        # 選択中のワークへ回収シーケンスを開始するだけのTriggerサービス
        # (2026-09-03追加、pick_sequence_confirmから「移動」部分を分離)。
        # 待機中の確定は行わない(それはpick_sequence_confirmの役目)。実際の
        # 処理はset_pick_move_handlerで登録されたコールバック(CommandGuiApp.
        # _on_pick_move_only_requested)に委譲する。PSコン側では×ボタン
        # (立ち上がりエッジ即時)から呼ばれる。
        self._pick_move_handler = None
        self.create_service(Trigger, 'pick_sequence_move', self._on_pick_sequence_move_srv)

        # 「L4へ移動」「R4へ移動」ボタン(固定のシューティングエリアへ、安全高度を
        # 維持したまま向かうだけの投入シーケンスを実行する)を、GUIボタンだけでなく
        # PSコン(joy_teleop_node)の割当ボタンからも呼べるようにするTriggerサービス
        # (2026-09-03、ユーザー指摘: 「シューティングエリアはL4もしくはR4で、
        # ボタンを2つおいておいて」)。pick_sequence_confirmと同じ構造で、実際の
        # 処理はset_shoot_start_handlerで登録されたコールバック
        # (CommandGuiApp._on_shoot_start_requested、対象ラベル引数付き)に委譲する。
        self._shoot_start_handler = None
        self.create_service(
            Trigger, 'shoot_sequence_start_l4',
            functools.partial(self._on_shoot_sequence_start_srv, 'L4'))
        self.create_service(
            Trigger, 'shoot_sequence_start_r4',
            functools.partial(self._on_shoot_sequence_start_srv, 'R4'))

        # 矢印キー(D-pad)でGUI上の目標ワーク選択カーソルを移動するTriggerサービス
        # 4つ(2026-09-03追加、ユーザー指定:「矢印キーでGUI上で目標ワークを選択し
        # 移動バツで移動、再度バツで回収実行」)。実際の処理はset_work_select_
        # handlerで登録されたコールバック(CommandGuiApp._on_work_select_requested、
        # 方向文字列引数付き)に委譲する。選択自体の移動であり、シーケンスの開始は
        # 行わない(開始は×ボタン=pick_sequence_confirm、_on_pick_confirm_requested
        # 参照)。
        self._work_select_handler = None
        for direction in ('up', 'down', 'left', 'right'):
            self.create_service(
                Trigger, f'select_work_{direction}',
                functools.partial(self._on_select_work_srv, direction))

        # ハンドのポンプON/OFF状態(hand_node、2026-09-03再追加)。「ピック/投入
        # 自動シーケンス」パネルにもポンプ操作ボタン・状態表示を出すために購読する
        # (joy_teleop_nodeの同トピック購読部と同じ理由・QoS)。
        self._pump_on_state = None
        pump_state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'hand_pump_state', self._on_pump_state, pump_state_qos)

        # ---- 状態表示灯(黄色/赤色LED)用の状態購読 (2026-09-08追加、
        # note/note_soki/can_mapping.txt「## 状態表示灯」参照) ----
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._homing_state = None  # homing_node.STATE_*文字列。未受信ならNone
        self.create_subscription(String, 'homing_state', self._on_homing_state, latched_qos)
        # z/rのMITゲイン自動調整(autotune_node)の状態・進捗表示用(2026-09-09追加)。
        self._autotune_state = None
        self._autotune_progress = None
        self.create_subscription(String, 'autotune_state', self._on_autotune_state, latched_qos)
        self.create_subscription(String, 'autotune_progress', self._on_autotune_progress, 10)
        self._estop_active = False
        self.create_subscription(Bool, 'estop_active', self._on_estop_active, latched_qos)
        self._limit_stop_active = False
        self.create_subscription(Bool, 'limit_stop_active', self._on_limit_stop_active, latched_qos)

        # 上記から計算した最終的なLED論理状態('off'/'on'/'blink_fast'/'blink_slow'、
        # LedIndicatorWidget.STATE_*と同じ値)をpublishする(2026-09-08追加)。
        # 実機のCAN_HOST(101)のMULTI1/MULTI2を駆動するのはhand_node.py
        # (pump/vacuum_releaseと同じdevice_id=101のCAN送信バッファを共有するため、
        # 送信元を1つにまとめる必要がある。詳細はhand_node.py _update_led_output・
        # note/note_soki/can_mapping.txt「## 状態表示灯」参照)。ここでは判定ロジック
        # の結果を配るだけで、実機へのCAN送信は行わない。
        self.led_yellow_state_pub_ = self.create_publisher(String, 'led_yellow_state', latched_qos)
        self.led_red_state_pub_ = self.create_publisher(String, 'led_red_state', latched_qos)

        # CAN_HOST(101)・CubeMars(11)・ROBOMAS(21)それぞれの生存監視(赤色LEDの
        # 「ノード未起動」判定用)。serial_rx_{device_id}_unwrappedの最終受信時刻を
        # 記録し、STATUS_DEVICE_STALE_TIMEOUT_SEC以上届いていなければ未起動とみなす
        # (ros2can側の接続判定と同じ「最近データが来ているか」の考え方)。
        self._device_last_seen_monotonic = {}
        # 上記の生存監視(最終受信時刻)に加え、CAN_HOST(101)については生スロット値
        # そのものも保持する(原点センサ配線確認パネル用、2026-09-08追加、
        # get_device_raw_slots参照)。
        self._device_last_data = {}
        for device_id in STATUS_DEVICE_IDS:
            self.create_subscription(
                Int32MultiArray, f'serial_rx_{device_id}_unwrapped',
                functools.partial(self._on_device_feedback, device_id), 10)

        # ---- ソフト緊急停止 (2026-09-08追加) ----
        # PSコン(joy_teleop_node)のPSボタン・GUIの「緊急停止」ボタンいずれからも
        # 呼べるTriggerサービス。実際の処理(自動シーケンス中断・ホーミング中断・
        # trajectory_follower_node出力凍結)はset_estop_handler/set_estop_clear_
        # handlerで登録されたCommandGuiAppのコールバックに委譲する
        # (pick_sequence_confirm等と同じ構造)。
        self._estop_handler = None
        self._estop_clear_handler = None
        self.create_service(Trigger, 'emergency_stop', self._on_emergency_stop_srv)
        self.create_service(Trigger, 'clear_emergency_stop', self._on_clear_emergency_stop_srv)

        # trajectory_follower_node/joy_teleop_nodeいずれのros2パラメータも同じ
        # get_parameters/set_parametersサービス経由でGUIから読込・変更できるよう、
        # 対象ノード名ごとにクライアントの組を保持する。
        self._param_clients = {
            node_name: (
                self.create_client(GetParameters, f'/{node_name}/get_parameters'),
                self.create_client(SetParameters, f'/{node_name}/set_parameters'),
            )
            for node_name in (TRAJ_NODE_NAME, JOY_NODE_NAME)
        }
        # std_srvs/Triggerサービス(/set_root_theta_origin等)呼び出し用クライアント。
        # サービス名ごとに遅延生成してキャッシュする。
        self._trigger_clients = {}

        # joy_teleop_nodeの手先θ追従トグル(OPTIONSボタン)をGUI側から明示的に
        # ON/OFFするクライアント(std_srvs/SetBool、2026-09-03追加)。投入(L4/R4)
        # シーケンス開始時にOFFへ切り替えるために使う(set_joy_tip_theta_follow・
        # CommandGuiApp._start_shoot_sequence参照。ユーザー指定:「手先θ追従は
        # シューティングボックスへの自動移動時には自動で無効化」)。
        self._joy_tip_theta_follow_client_ = self.create_client(
            SetBool, 'set_tip_theta_follow_theta')

    def _on_mixed_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_positions:
                self._current_positions[name] = pos
        self._current_received = True

    def has_current_state(self):
        return self._current_received

    def _on_pump_state(self, msg):
        self._pump_on_state = msg.data

    def get_pump_on_state(self):
        """ポンプの現在ON/OFF状態(hand_pump_state購読)。まだ受信していなければNone。"""
        return self._pump_on_state

    def _on_homing_state(self, msg):
        self._homing_state = msg.data

    def get_homing_state(self):
        """homing_node.STATE_*文字列(idle/homing_z/homing_r/done/failed)。
        homing_node未起動でまだ受信していなければNone。"""
        return self._homing_state

    def _on_autotune_state(self, msg):
        self._autotune_state = msg.data

    def get_autotune_state(self):
        """autotune_node.STATE_*文字列(idle/running_z/running_r/done/failed)。
        autotune_node未起動でまだ受信していなければNone。"""
        return self._autotune_state

    def _on_autotune_progress(self, msg):
        self._autotune_progress = msg.data

    def get_autotune_progress(self):
        """autotune_nodeの直近の進捗メッセージ(試行ごとのkp/kd/score等)。
        未受信ならNone。"""
        return self._autotune_progress

    def _on_estop_active(self, msg):
        self._estop_active = msg.data

    def get_estop_active(self):
        """trajectory_follower_nodeが緊急停止で出力を凍結中かどうか
        (estop_active購読)。未受信ならFalse(trajectory_follower_node未起動時は
        そもそも出力自体が無いため安全側)。"""
        return self._estop_active

    def _on_limit_stop_active(self, msg):
        self._limit_stop_active = msg.data

    def get_limit_stop_active(self):
        """z/rいずれかのリミットスイッチが現在トリガーされ安全停止中かどうか
        (limit_stop_active購読)。"""
        return self._limit_stop_active

    def _on_device_feedback(self, device_id, msg):
        self._device_last_seen_monotonic[device_id] = time.monotonic()
        self._device_last_data[device_id] = msg.data

    def get_device_raw_slots(self, device_id):
        """serial_rx_{device_id}_unwrappedの最新の生スロット値(Int32MultiArray.data)。
        未受信ならNone(原点センサ配線確認パネル用、2026-09-08追加)。"""
        return self._device_last_data.get(device_id)

    def get_stale_device_ids(self):
        """STATUS_DEVICE_IDSのうち、STATUS_DEVICE_STALE_TIMEOUT_SEC以上
        serial_rx_{id}_unwrappedが届いていない(=一度も受信していない場合を含む)
        deviceのidリストを返す(赤色LEDの「ノード未起動」判定用)。"""
        now = time.monotonic()
        stale = []
        for device_id in STATUS_DEVICE_IDS:
            last_seen = self._device_last_seen_monotonic.get(device_id)
            if last_seen is None or (now - last_seen) >= STATUS_DEVICE_STALE_TIMEOUT_SEC:
                stale.append(device_id)
        return stale

    def publish_led_states(self, yellow, red):
        """状態表示灯の最終ロジック状態(LedIndicatorWidget.STATE_*文字列)を
        led_yellow_state/led_red_stateへpublishする(2026-09-08追加、実機の
        LEDはhand_node.pyが購読して駆動する。CommandGuiApp._update_status_leds
        から呼ばれる)。"""
        msg = String()
        msg.data = yellow
        self.led_yellow_state_pub_.publish(msg)
        msg = String()
        msg.data = red
        self.led_red_state_pub_.publish(msg)

    def set_estop_handler(self, handler):
        """CommandGuiApp._on_emergency_stop_requested(引数無し、戻り値無し)を
        登録する(2026-09-08追加)。"""
        self._estop_handler = handler

    def _on_emergency_stop_srv(self, request, response):
        if self._estop_handler is None:
            response.success = False
            response.message = 'GUI未初期化です'
            return response
        self._estop_handler()
        response.success = True
        response.message = '緊急停止しました'
        return response

    def set_estop_clear_handler(self, handler):
        """CommandGuiApp._on_clear_emergency_stop_requested(引数無し、戻り値無し)を
        登録する(2026-09-08追加)。"""
        self._estop_clear_handler = handler

    def _on_clear_emergency_stop_srv(self, request, response):
        if self._estop_clear_handler is None:
            response.success = False
            response.message = 'GUI未初期化です'
            return response
        self._estop_clear_handler()
        response.success = True
        response.message = '緊急停止を解除しました'
        return response

    def set_pick_confirm_handler(self, handler):
        """CommandGuiApp._on_pick_confirm_only_requested(引数無し、bool返却)を
        登録する(2026-09-03、PSボタンでのみ呼ばれる確定専用ハンドラ)。"""
        self._pick_confirm_handler = handler

    def _on_pick_sequence_confirm_srv(self, request, response):
        if self._pick_confirm_handler is None:
            response.success = False
            response.message = 'GUI未初期化です'
            return response
        response.success = self._pick_confirm_handler()
        response.message = (
            '回収を実行します' if response.success else '現在「回収実行」待ちの状態ではありません')
        return response

    def set_pick_move_handler(self, handler):
        """CommandGuiApp._on_pick_move_only_requested(引数無し、bool返却)を
        登録する(2026-09-03追加、×短押しで呼ばれる移動専用ハンドラ)。"""
        self._pick_move_handler = handler

    def _on_pick_sequence_move_srv(self, request, response):
        if self._pick_move_handler is None:
            response.success = False
            response.message = 'GUI未初期化です'
            return response
        response.success = self._pick_move_handler()
        response.message = (
            '選択中のワークへ移動します' if response.success else '移動できません(選択ワーク未確定?)')
        return response

    def set_shoot_start_handler(self, handler):
        """CommandGuiApp._on_shoot_start_requested(label: str、bool返却)を登録する。"""
        self._shoot_start_handler = handler

    def _on_shoot_sequence_start_srv(self, label, request, response):
        if self._shoot_start_handler is None:
            response.success = False
            response.message = 'GUI未初期化です'
            return response
        response.success = self._shoot_start_handler(label)
        response.message = f'{label}へ移動します' if response.success else f'{label}への移動に失敗しました'
        return response

    def set_work_select_handler(self, handler):
        """CommandGuiApp._on_work_select_requested(direction: 'up'/'down'/'left'/
        'right'、戻り値無し)を登録する。"""
        self._work_select_handler = handler

    def _on_select_work_srv(self, direction, request, response):
        if self._work_select_handler is None:
            response.success = False
            response.message = 'GUI未初期化です'
            return response
        self._work_select_handler(direction)
        response.success = True
        response.message = f'ワーク選択カーソルを{direction}へ移動しました'
        return response

    def set_joy_tip_theta_follow(self, enabled):
        """joy_teleop_nodeの手先θ追従トグルをGUI側から明示的にON/OFFする
        (std_srvs/SetBool、2026-09-03追加)。投入(L4/R4)シーケンス開始時にOFFへ
        切り替えるために使う(CommandGuiApp._start_shoot_sequence参照)。
        joy_teleop_node未起動時は黙って諦める(安全側、シーケンス自体は
        続行してよいため呼び出し元をブロックしない)。"""
        if not self._joy_tip_theta_follow_client_.service_is_ready():
            return
        req = SetBool.Request()
        req.data = enabled
        self._joy_tip_theta_follow_client_.call_async(req)

    def get_active_node_names(self):
        """現在ROSグラフに存在するノード名の集合を返す(機体ステータスパネルの
        起動状況表示用)。"""
        return set(self.get_node_names())

    def get_current_positions(self):
        return dict(self._current_positions)

    def send_target(self, theta, zj, r, tip_theta=None):
        """tip_theta(手先θ)はNoneなら含めない(=trajectory_follower_node側が
        保持している現在の目標のまま、他の呼び出し元(手動XY移動・ジョグ等)は
        従来通りtip_thetaに触れない)。ピック/投入シーケンスのみワークの行との
        平行/シュート方向との垂直を保つためtip_thetaも指定する
        (2026-09-03、_start_pick_sequence/_start_shoot_sequence参照)。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'auto'
        if tip_theta is None:
            msg.name = list(JOINT_NAMES)
            msg.position = [theta, zj, r]
        else:
            msg.name = list(JOINT_NAMES) + ['tip_theta_joint']
            msg.position = [theta, zj, r, tip_theta]
        self.pub_.publish(msg)

    def send_machine_origin(self, x, y, z):
        """機体原点オフセット(machine_origin_x/y/z_joint)を/mixed_joint_statesへ
        直接publishする。trajectory_follower_nodeを経由しないため即座に反映される。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MACHINE_ORIGIN_JOINT_NAMES)
        msg.position = [x, y, z]
        self.mixed_pub_.publish(msg)

    def send_hand_offset(self, x, y, z):
        """ハンド取付オフセット(hand_offset_x/y/z_joint)を/mixed_joint_statesへ
        直接publishする(send_machine_originと同じ理由・方式)。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(HAND_OFFSET_JOINT_NAMES)
        msg.position = [x, y, z]
        self.mixed_pub_.publish(msg)

    def request_node_params(self, target_node, names, on_success, on_failure=None):
        """target_node(trajectory_follower_nodeまたはjoy_teleop_node)のパラメータを
        非同期取得する。

        on_success({name: value})/on_failure(str)は、後続のrclpy.spin_once()
        呼び出し中(QTimerのタイムアウトループと同じメインスレッド)に呼ばれる。"""
        get_cli, _ = self._param_clients[target_node]
        if not get_cli.service_is_ready():
            return False
        future = get_cli.call_async(GetParameters.Request(names=names))

        def _done(fut):
            try:
                res = fut.result()
            except Exception as exc:
                if on_failure:
                    on_failure(str(exc))
                return
            values = {}
            for name, pv in zip(names, res.values):
                if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    values[name] = list(pv.double_array_value)
                elif pv.type == ParameterType.PARAMETER_DOUBLE:
                    values[name] = pv.double_value
                elif pv.type == ParameterType.PARAMETER_INTEGER:
                    values[name] = pv.integer_value
                elif pv.type == ParameterType.PARAMETER_STRING:
                    values[name] = pv.string_value
                elif pv.type == ParameterType.PARAMETER_STRING_ARRAY:
                    values[name] = list(pv.string_array_value)
                elif pv.type == ParameterType.PARAMETER_BOOL:
                    values[name] = pv.bool_value
            on_success(values)

        future.add_done_callback(_done)
        return True

    def set_node_params(self, target_node, values, on_done=None):
        """values: {name: [float,...] または float またはstr} をtarget_nodeへ非同期set。

        on_done(list[SetParametersResult] または None)は、後続のrclpy.spin_once()
        呼び出し中(QTimerのタイムアウトループと同じメインスレッド)に呼ばれる。"""
        _, set_cli = self._param_clients[target_node]
        if not set_cli.service_is_ready():
            return False
        params = []
        for name, v in values.items():
            if isinstance(v, (list, tuple)):
                # 空配列はdisabled_joints(文字列配列)の「全軸有効」を送る場合に
                # 発生するため、要素の有無だけでは型を判別できない。全要素が
                # strなら(空配列含め)文字列配列として扱う。
                if all(isinstance(x, str) for x in v):
                    pv = ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                                         string_array_value=list(v))
                else:
                    pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                         double_array_value=[float(x) for x in v])
            elif isinstance(v, str):
                pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
            elif isinstance(v, bool):
                # bool は int のサブクラスなので、下のfloat(v)分岐より先に判定する
                # 必要がある(そうしないとPARAMETER_DOUBLEとして送ってしまい、
                # ノード側がboolとして宣言したパラメータへの型不一致で拒否される)。
                pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
            else:
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            params.append(Parameter(name=name, value=pv))
        future = set_cli.call_async(SetParameters.Request(parameters=params))

        if on_done:
            def _done(fut):
                try:
                    res = fut.result()
                except Exception:
                    on_done(None)
                    return
                on_done(res.results)

            future.add_done_callback(_done)
        return True

    def call_trigger_service(self, service_name, on_done):
        """std_srvs/Trigger型のサービス(/set_root_theta_origin等)を非同期呼び出しする。

        on_done(success: bool, message: str)は、後続のrclpy.spin_once()呼び出し中
        (QTimerのタイムアウトループと同じメインスレッド)に呼ばれる。サービス未起動の
        場合は即座にon_done(False, ...)を呼んでFalseを返す。"""
        client = self._trigger_clients.get(service_name)
        if client is None:
            client = self.create_client(Trigger, service_name)
            self._trigger_clients[service_name] = client
        if not client.service_is_ready():
            on_done(False, f'{service_name} が起動していません')
            return False
        future = client.call_async(Trigger.Request())

        def _done(fut):
            try:
                res = fut.result()
            except Exception as exc:
                on_done(False, str(exc))
                return
            on_done(res.success, res.message)

        future.add_done_callback(_done)
        return True


class CommandGuiApp(QWidget):
    CANVAS_SIZE = 320
    MARGIN = 16
    Z_SLIDER_SCALE = 1000  # QSliderは整数値のみのため、mm単位の整数で表現する
    JOINT_SLIDER_SCALE = 1000  # 関節スライダー(_build_joint_slider_panel)用。
                                # rad/mどちらも1/1000刻み(約0.057deg、1mm相当)で
                                # 手動ジョグには十分な分解能。

    def __init__(self, node: CommandGuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('soki_sim command GUI')

        icon_pixmap = _load_soki_logo_image(target_height=64)
        if icon_pixmap is not None:
            self.setWindowIcon(QIcon(icon_pixmap))

        self.points = self._load_points()
        # trajectory/MITとも、対象joint名は本来ノード側の実行構成次第だが、
        # 「読込」を経なくても起動直後からGUI保持値をそのまま適用できるように、
        # GUI側で既知の定数(JOINT_NAMES/CUBEMARS_JOINT_NAMES)を初期値にしておく。
        # 「読込」を実行した場合はノードの実際の構成(一部関節のみ等)で上書きされる。
        self._traj_joint_names = list(JOINT_NAMES)
        # tip_theta_joint等、JOINT_NAMES(GUIの軌道パネルが編集欄を持つ関節)に
        # 含まれない関節がノード側のjoint_namesに含まれる場合(real_all_axes_test.
        # launch.py等)に、その関節の最後に読み込んだ値を保持しておく。GUIに編集欄が
        # 無いため「適用」時にその関節の値だけ書き変えずに送り直すために使う
        # (無いと_collect_traj_valuesがKeyErrorになり、実際は数値の問題ではないのに
        # 「速度・加速度に数値を入力してください」と誤表示される)。
        self._traj_loaded_extra = {}
        self._mit_joint_names = list(CUBEMARS_JOINT_NAMES)
        self._saved_gains = self._load_gains_file()
        self._apply_all_results = {}
        self._mode_buttons = {}
        # 実機セットアップパネルからros2 launchで起動する子プロセス(未起動ならNone)。
        self._launch_process = None

        # ピック/投入自動シーケンスの実行状態(_advance_sequence参照)。
        self._seq_active = False
        self._seq_steps = []
        self._seq_index = 0
        self._seq_kind = None
        self._seq_waiting_service = None
        self._seq_leg_target = None
        self._seq_leg_start_time = None
        self._seq_delay_start_time = None
        # 直近の回収対象(x, y, z)。吸着に失敗した場合など、シーケンス完了/中断後
        # でも「回収実行」を再度押すだけで同じワークへやり直せるようにするため
        # 記憶しておく(2026-09-03、ユーザー指摘: 「吸着できなかったときに回収
        # 実行を再度行うことがある。現状だと回収実行が一回しかできない」)。
        self._last_pick_target = None
        # X/Y/Z編集欄は起動時点では固定デフォルト値(可動域中央付近)のままで
        # 実機の現在位置とは無関係なため、電源投入直後にうっかり「送信」を押すと
        # そのデフォルト値へ向けて急に動き出してしまう。電源off/on後もGUIを
        # 起動し直すたびに再発するため、_refresh_current_stateで実機の現在位置を
        # 最初に受信した時点で一度だけX/Y/Z欄を現在位置へ自動同期する
        # (_on_copy_current_to_targetと同じ変換)。以後はユーザーの手動編集を
        # 尊重し上書きしない。
        self._target_synced_to_current_ = False

        self.x_edit = make_float_edit(MAX_RADIUS / 2.0)
        self.y_edit = make_float_edit(0.0)
        self.z_edit = make_float_edit((WORLD_Z_LOWER + WORLD_Z_UPPER) / 2.0)
        self.step_edit = make_float_edit(0.01, width=60)

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(8, 8, 8, 8)

        # パラメータパネルが増えるにつれ縦に伸び、ワーク/シューティングボックスの
        # ボタン(元は座標指定操作タブ側)が画面外に押し出される問題が起きたため、
        # QTabWidgetで機能ごとにタブを分離している。「座標指定操作」(円クリック・
        # ジョグ・保存済みポイント等、自由な位置への移動系)、「統合操作」(現在状態・
        # 動作モード・ワーク/シューティングボックスの定型位置移動ボタン)、
        # 「ゲイン調整」(軌道生成・joy速度・MIT・robomasの各ゲインと全ゲイン一括
        # 読込/適用)、「原点校正」(機体原点オフセット・root_theta原点設定・
        # ホーミング)、「配線設定」(robomas/cubemars/homing/リミットスイッチの
        # real_joint_bridge.yaml配線設定)の5タブ構成。起動時に表示するタブは、
        # 動作モード切替など主要な操作をまとめた統合操作タブをデフォルトにする。
        logo_pixmap = _load_soki_logo_image(target_height=36)
        if logo_pixmap is not None:
            header = QHBoxLayout()
            header.addStretch(1)
            logo_label = QLabel()
            logo_label.setPixmap(logo_pixmap)
            header.addWidget(logo_label)
            root_layout.addLayout(header)

        # ---- 状態表示灯・緊急停止バー (2026-09-08新規) ----
        # 実機CAN_HOST(device_id=101、MULTI1=黄色LED/MULTI2=赤色LED)の状態表示灯を
        # 模した表示と、ソフト緊急停止の操作をどのタブを開いていても見える位置に置く
        # (note/note_soki/can_mapping.txt「## 状態表示灯」参照)。
        estop_bar = QHBoxLayout()
        estop_bar.addWidget(QLabel('状態表示灯:'))
        estop_bar.addWidget(QLabel('注意'))
        self.yellow_led = LedIndicatorWidget('#f4b400')
        estop_bar.addWidget(self.yellow_led)
        estop_bar.addWidget(QLabel('異常'))
        self.red_led = LedIndicatorWidget('#d93025')
        estop_bar.addWidget(self.red_led)
        estop_bar.addSpacing(16)
        self.estop_status_label = QLabel()
        self.estop_status_label.setWordWrap(True)
        _set_status(self.estop_status_label, '', 'muted')
        estop_bar.addWidget(self.estop_status_label, 1)
        estop_btn = QPushButton('緊急停止')
        estop_btn.setProperty('variant', 'danger')
        estop_btn.clicked.connect(self._on_emergency_stop_requested)
        estop_bar.addWidget(estop_btn)
        estop_clear_btn = QPushButton('解除')
        estop_clear_btn.clicked.connect(self._on_clear_emergency_stop_requested)
        estop_bar.addWidget(estop_clear_btn)
        root_layout.addLayout(estop_bar)

        self.tabs = QTabWidget()
        root_layout.addWidget(self.tabs)

        manual_tab = QWidget()
        overview_tab = QWidget()
        gain_tab = QWidget()
        calibration_tab = QWidget()
        wiring_tab = QWidget()
        status_display_tab = QWidget()
        # 統合操作を既定表示にするだけでなく、タブの並びも一番左にする
        # (先に追加した方が左側になる)。状態表示(拡大)はスマホでの画面共有
        # 閲覧用(操作ボタンは置かず表示専用)のため末尾に追加するだけでよい。
        self.tabs.addTab(overview_tab, '統合操作')
        self.tabs.addTab(gain_tab, 'ゲイン調整')
        self.tabs.addTab(calibration_tab, '原点校正')
        self.tabs.addTab(wiring_tab, '配線設定')
        self.tabs.addTab(manual_tab, '座標指定操作')
        self.tabs.addTab(status_display_tab, '状態表示(拡大)')

        self._build_move_tab(manual_tab)
        self._build_overview_tab(overview_tab)
        self._build_gain_tab(gain_tab)
        self._build_calibration_tab(calibration_tab)
        self._build_wiring_tab(wiring_tab)
        self._build_status_display_tab(status_display_tab)
        self._restore_saved_gains()
        # PSコン(joy_teleop_node)から、PSボタン=/pick_sequence_confirm
        # (確定のみ)・×ボタン=/pick_sequence_move(選択中ワークへの移動のみ)の
        # 2つのサービス経由で呼ばれる(2026-09-03、ユーザー指定:「回収ボタンを
        # バツ長押しからPSボタンに変更」。誤操作でワークに接触・吸着してしまう
        # ことを防ぐため、確定操作は移動用の×ボタンとは別ボタンにしている)。
        self.node.set_pick_confirm_handler(self._on_pick_confirm_only_requested)
        self.node.set_pick_move_handler(self._on_pick_move_only_requested)
        # PSコン(joy_teleop_node)の割当ボタンから/shoot_sequence_start_l4・_r4
        # サービス経由で呼ばれた際、GUIの「L4へ移動」「R4へ移動」ボタンと同じ
        # 処理を行わせる(2026-09-03)。
        self.node.set_shoot_start_handler(self._on_shoot_start_requested)
        # PSコン(joy_teleop_node)の十字キー(D-pad)から/select_work_up・_down・
        # _left・_rightサービス経由で呼ばれた際、ワーク選択カーソルを動かす
        # (2026-09-03、ユーザー指定:「矢印キーでGUI上で目標ワークを選択し移動」)。
        self.node.set_work_select_handler(self._on_work_select_requested)
        # PSコン(joy_teleop_node)のPSボタンから/emergency_stop・/clear_emergency_stop
        # サービス経由で呼ばれた際、GUIの「緊急停止」「解除」ボタンと同じ処理を行わせる
        # (2026-09-08追加)。
        self.node.set_estop_handler(self._on_emergency_stop_requested)
        self.node.set_estop_clear_handler(self._on_clear_emergency_stop_requested)

        self.tabs.setCurrentWidget(overview_tab)

        for edit in (self.x_edit, self.y_edit, self.z_edit):
            edit.textChanged.connect(self._redraw_pin)
        self._redraw_pin()
        self._refresh_point_list()

        # ゲイン調整タブは軌道生成/MIT/robomasゲイン等のグリッドが左右2列に並ぶため、
        # 940pxだとまだ右端が少し欠けて横スクロールが発生していた。実測で2列とも
        # 横スクロール無しで収まる幅(約1160px)を既定値にする。
        self.setMinimumWidth(660)
        self.resize(1180, 820)

        # rclpy.spin_once()はmixed_joint_states購読・パラメータサービスの応答処理に
        # 必要(このタイマーのコールバック=Qtのイベントループと同じメインスレッド上で
        # 完結する。別スレッドのexecutorにしない理由はモジュールdocstring参照)。
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros)
        self._spin_timer.start(50)

        # 起動時に、前回「全ノード起動」したlaunchプロセスが生き残っていないか
        # 確認する(ALL_AXES_LAUNCH_NODE_NAMES定義部のコメント参照)。discoveryに
        # 時間がかかるため、起動直後ではなく少し待ってから1回だけ確認する。
        QTimer.singleShot(1500, self._check_existing_launch_nodes)

        # trajectory_follower_node/joy_teleop_nodeがGUIより後に立ち上がることも
        # あるため、各サービスが使えるようになるまで一定間隔でリトライし、使えた
        # 時点でそれぞれ一度だけ自動読込・自動適用する(_try_auto_setup_gains参照)。
        # 「読込」は表示の同期のみ(joint_names等、実際のノード構成を知るため)。
        # 「適用」はgains.json(起動時にGUIへ復元済みの値、_restore_saved_gains参照)を
        # 実機へ自動でSetParametersする。手動で「適用」を押さない限りゲインが
        # 反映されずrobomas(z/r)が動かない、という問題(2026-09-02報告)への対処。
        # 手動の「適用」ボタンと違い確認ダイアログは出さない(起動のたびに人手を
        # 挟むと運用上煩雑なため)。
        self._traj_auto_loaded = False
        self._mit_auto_loaded = False
        self._robomas_auto_loaded = False
        self._robomas_vel_auto_loaded = False
        self._joy_auto_loaded = False
        # _traj_auto_loaded/_mit_auto_loadedは読込「リクエスト送信済み」を表すだけ
        # (request_node_paramsは非同期のため)。実際に_traj_joint_names/
        # _mit_joint_namesが応答で更新されたかは以下の別フラグで判定する
        # (_apply_loaded_traj_params/_apply_loaded_mit_gains参照)。
        self._traj_names_known = False
        self._mit_names_known = False
        self._traj_auto_applied = False
        self._mit_auto_applied = False
        self._robomas_auto_applied = False
        self._robomas_vel_auto_applied = False
        self._joy_auto_applied = False
        self._auto_load_timer = QTimer(self)
        self._auto_load_timer.timeout.connect(self._try_auto_setup_gains)
        self._auto_load_timer.start(500)

        # 統合操作タブの機体ステータスパネル(ノード起動状況・適用ゲイン・校正状態・
        # 実機セットアップの全ノード起動プロセス)を1秒間隔でポーリング更新する。
        self._refresh_machine_status()
        self._machine_status_timer = QTimer(self)
        self._machine_status_timer.timeout.connect(self._refresh_machine_status)
        self._machine_status_timer.start(1000)

    # ---------- manual tab ----------
    def _build_move_tab(self, parent):
        layout = QHBoxLayout(parent)

        left = QVBoxLayout()
        left.addWidget(QLabel('XY平面 (クリックでピン設置。上=ワーク側(Y+)、右=X+)'))
        self.xy_widget = XYPlaneWidget(self.CANVAS_SIZE, MAX_RADIUS, self.MARGIN, self._on_canvas_click)
        left.addWidget(self.xy_widget)
        left.addStretch(1)
        layout.addLayout(left)

        mid = QVBoxLayout()

        coord_box = QGroupBox('目標座標 [m]')
        coord_grid = QGridLayout(coord_box)
        for i, (label, edit) in enumerate((('X', self.x_edit), ('Y', self.y_edit), ('Z', self.z_edit))):
            coord_grid.addWidget(QLabel(label), i, 0)
            coord_grid.addWidget(edit, i, 1)
        copy_btn = QPushButton('現在位置を目標にコピー')
        copy_btn.clicked.connect(self._on_copy_current_to_target)
        coord_grid.addWidget(copy_btn, 3, 0, 1, 2)
        mid.addWidget(coord_box)

        jog_box = QGroupBox('ジョグ (↑ワーク側 ↓機体側 ←X- →X+)')
        jog_layout = QVBoxLayout(jog_box)
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel('ステップ[m]'))
        step_row.addWidget(self.step_edit)
        step_row.addStretch(1)
        jog_layout.addLayout(step_row)
        pad = QGridLayout()
        up_btn = QPushButton('↑')
        down_btn = QPushButton('↓')
        left_btn = QPushButton('←')
        right_btn = QPushButton('→')
        up_btn.clicked.connect(lambda: self._on_jog(0, 1))
        down_btn.clicked.connect(lambda: self._on_jog(0, -1))
        left_btn.clicked.connect(lambda: self._on_jog(-1, 0))
        right_btn.clicked.connect(lambda: self._on_jog(1, 0))
        for b in (up_btn, down_btn, left_btn, right_btn):
            b.setFixedWidth(36)
        pad.addWidget(up_btn, 0, 1)
        pad.addWidget(left_btn, 1, 0)
        pad.addWidget(right_btn, 1, 2)
        pad.addWidget(down_btn, 2, 1)
        jog_layout.addLayout(pad)
        mid.addWidget(jog_box)

        mid.addWidget(QLabel(f'Z [{WORLD_Z_LOWER:.2f} - {WORLD_Z_UPPER:.2f} m]'))
        self.z_slider = QSlider(Qt.Vertical)
        self.z_slider.setMinimum(int(round(WORLD_Z_LOWER * self.Z_SLIDER_SCALE)))
        self.z_slider.setMaximum(int(round(WORLD_Z_UPPER * self.Z_SLIDER_SCALE)))
        self.z_slider.setSingleStep(5)
        self.z_slider.setValue(int(round(get_float(self.z_edit) * self.Z_SLIDER_SCALE)))
        self.z_slider.setFixedHeight(180)
        self.z_slider.valueChanged.connect(self._on_z_slider_changed)
        self.z_edit.textChanged.connect(self._on_z_edit_changed)
        mid.addWidget(self.z_slider, alignment=Qt.AlignHCenter)

        self.status_label = QLabel('')
        self.status_label.setWordWrap(True)
        mid.addWidget(self.status_label)

        send_btn = QPushButton('送信 (Send)')
        send_btn.setProperty('variant', 'primary')
        send_btn.clicked.connect(self._on_send)
        mid.addWidget(send_btn)
        mid.addStretch(1)
        layout.addLayout(mid)

        right = QVBoxLayout()
        right.addWidget(QLabel('保存済みポイント (ダブルクリックで読込)'))
        self.listbox = QListWidget()
        self.listbox.itemDoubleClicked.connect(lambda _item: self._on_load_point())
        right.addWidget(self.listbox)
        btns = QHBoxLayout()
        add_btn = QPushButton('追加')
        send_sel_btn = QPushButton('送信')
        del_btn = QPushButton('削除')
        add_btn.clicked.connect(self._on_add_point)
        send_sel_btn.clicked.connect(self._on_send_selected)
        del_btn.clicked.connect(self._on_delete_point)
        for b in (add_btn, send_sel_btn, del_btn):
            btns.addWidget(b)
        right.addLayout(btns)
        layout.addLayout(right, 1)

        joints = QVBoxLayout()
        self._build_joint_slider_panel(joints)
        joints.addStretch(1)
        layout.addLayout(joints)

    def _build_joint_slider_panel(self, layout):
        """関節ごと(root_theta/tip_theta/z/r)に直接スライダーでジョグできる
        パネル(2026-09-09追加、ユーザー要望: 「コントローラー操作は一旦おいて、
        各軸をスライダーで操作できるように。ホーミング後にスライドバーの位置と
        実機の位置が合うように」)。左のXY平面/座標入力(直交座標、IK経由)とは
        独立な関節空間の直接操作。

        「ホーミング後に実機の位置と合う」ようにするため、_refresh_joint_sliders
        (_machine_status_timerとは別の専用QTimer、100ms間隔)がユーザーが
        ドラッグ中でない(isSliderDown()==False)スライダーへ常時
        get_current_positions()の値を反映し続ける(ホーミング後に限らず常時。
        homing_nodeがoffsetを適用してmixed_joint_statesの値が飛んだ場合も、
        次のタイマー周期で自動的に追従する)。この反映はblockSignals()で行うため
        valueChangedは発火せず、ユーザーの実際の操作(ドラッグ・クリック・
        キーボード)によるvalueChangedとは自己フィードバックなく区別できる。"""
        box = QGroupBox('関節スライダー (直接ジョグ)')
        col = QVBoxLayout(box)

        note = QLabel()
        note.setWordWrap(True)
        _set_status(
            note,
            'ドラッグ中でない間は実機の現在値に自動追従する(ホーミング直後の\n'
            '位置反映もここで行われる)。ドラッグ・クリック・キー操作で目標値を送信。',
            'muted')
        col.addWidget(note)

        joint_slider_specs = (
            ('root_theta_joint', 'root θ', ROOT_THETA_LOWER, ROOT_THETA_UPPER, 'rad'),
            ('tip_theta_joint', 'tip θ', TIP_THETA_LOWER, TIP_THETA_UPPER, 'rad'),
            ('z_joint', 'z', Z_LOWER, Z_UPPER, 'm'),
            ('r_joint', 'r', R_LOWER, R_UPPER, 'm'),
        )
        self.joint_sliders = {}
        self.joint_slider_value_labels = {}
        for name, label, lower, upper, unit in joint_slider_specs:
            row = QHBoxLayout()
            name_label = QLabel(label)
            name_label.setFixedWidth(36)
            row.addWidget(name_label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(round(lower * self.JOINT_SLIDER_SCALE)))
            slider.setMaximum(int(round(upper * self.JOINT_SLIDER_SCALE)))
            slider.setFixedWidth(160)
            value_label = QLabel('-')
            value_label.setFixedWidth(64)
            slider.valueChanged.connect(
                lambda value, n=name, u=unit: self._on_joint_slider_changed(n, value, u))
            self.joint_sliders[name] = slider
            self.joint_slider_value_labels[name] = value_label
            row.addWidget(slider)
            row.addWidget(value_label)
            col.addLayout(row)

        layout.addWidget(box)

        self._joint_slider_timer = QTimer(self)
        self._joint_slider_timer.timeout.connect(self._refresh_joint_sliders)
        self._joint_slider_timer.start(100)

    def _refresh_joint_sliders(self):
        if not self.node.has_current_state():
            return
        pos = self.node.get_current_positions()
        for name, slider in self.joint_sliders.items():
            if slider.isSliderDown():
                continue
            value = clamp(int(round(pos[name] * self.JOINT_SLIDER_SCALE)),
                          slider.minimum(), slider.maximum())
            if value == slider.value():
                continue
            slider.blockSignals(True)
            slider.setValue(value)
            slider.blockSignals(False)
            self._set_joint_slider_label(name)

    def _set_joint_slider_label(self, name):
        unit = 'rad' if name in ('root_theta_joint', 'tip_theta_joint') else 'm'
        value = self.joint_sliders[name].value() / self.JOINT_SLIDER_SCALE
        self.joint_slider_value_labels[name].setText(f'{value:.3f}{unit}')

    def _on_joint_slider_changed(self, name, _value, _unit):
        # _refresh_joint_slidersからの反映はblockSignals()で行っているため、
        # ここに来るのは常にユーザーの実操作(ドラッグ・クリック・キー)。
        self._set_joint_slider_label(name)
        theta = self.joint_sliders['root_theta_joint'].value() / self.JOINT_SLIDER_SCALE
        tip_theta = self.joint_sliders['tip_theta_joint'].value() / self.JOINT_SLIDER_SCALE
        z = self.joint_sliders['z_joint'].value() / self.JOINT_SLIDER_SCALE
        r = self.joint_sliders['r_joint'].value() / self.JOINT_SLIDER_SCALE
        self.node.send_target(theta, z, r, tip_theta)

    def _on_z_slider_changed(self, value):
        z = value / self.Z_SLIDER_SCALE
        self.z_edit.blockSignals(True)
        set_float(self.z_edit, z)
        self.z_edit.blockSignals(False)
        self._redraw_pin()

    def _on_z_edit_changed(self, _text):
        try:
            z = get_float(self.z_edit)
        except ValueError:
            return
        value = clamp(int(round(z * self.Z_SLIDER_SCALE)), self.z_slider.minimum(), self.z_slider.maximum())
        self.z_slider.blockSignals(True)
        self.z_slider.setValue(value)
        self.z_slider.blockSignals(False)

    # ---------- panel tabs (統合操作・ゲイン調整・原点校正・配線設定) ----------
    def _build_panel_tab(self, parent, top_funcs=(), left_funcs=(), right_funcs=()):
        """指定したパネル構築関数群(いずれも_build_current_state_panel等、column用の
        QVBoxLayoutを1つ受け取る形式)を、スクロール可能な領域に配置する共通ヘルパー。
        元は統合操作タブ1枚に全パネルを詰めていたが、機能ごとにタブを分割する際の
        重複を避けるため関数化した。right_funcsを渡さない場合は単一列になる。"""
        outer = QVBoxLayout(parent)
        outer.setContentsMargins(0, 0, 0, 0)

        # パネルが増えても画面からはみ出さないよう、スクロール可能な領域に入れる。
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        inner_layout = QVBoxLayout(inner)

        for func in top_funcs:
            func(inner_layout)

        # ストレッチ比を指定しないとQBoxLayoutは余った横幅を両列に均等配分して
        # しまい、内容が小さい列だけ不自然に広がっていた。かといって片方の列に
        # stretch=1を与えると、今度はその列内のQGridLayout(ラベル列)が余白を
        # 吸ってラベルと入力欄の間に大きな隙間ができてしまう。どちらの列も
        # 中身ぴったりのサイズに留め、余った横幅は列の外側(末尾のstretch)へ
        # 逃がすのが正しい。
        columns = QHBoxLayout()
        left_col = QVBoxLayout()
        columns.addLayout(left_col, 0)
        if right_funcs:
            right_col = QVBoxLayout()
            columns.addLayout(right_col, 0)
        columns.addStretch(1)
        inner_layout.addLayout(columns)
        inner_layout.addStretch(1)

        for func in left_funcs:
            func(left_col)
        # 同様に縦方向も、もう片方の列に合わせて各パネルが間延びして伸びないよう、
        # 余った縦幅は末尾のstretchへ逃がす。
        left_col.addStretch(1)

        if right_funcs:
            for func in right_funcs:
                func(right_col)
            right_col.addStretch(1)

        scroll.setWidget(inner)
        outer.addWidget(scroll)

    def _build_overview_tab(self, parent):
        self._build_panel_tab(
            parent,
            top_funcs=[self._build_field_buttons],
            left_funcs=[self._build_machine_status_panel, self._build_current_state_panel,
                        self._build_mode_panel, self._build_hand_panel],
            right_funcs=[self._build_setup_checklist_panel])

    # role色(_ROLE_COLORS)の文字色を、遠目にも分かる塗りつぶしカード(背景色+
    # 文字色)に変換するテーブル(状態表示(拡大)タブ専用。統合操作タブ側は文字色
    # だけの従来表示のまま変更しない)。
    _STATUS_BLOCK_STYLES = {
        _ROLE_COLORS['info']: ('#1a73e8', '#ffffff'),
        _ROLE_COLORS['success']: ('#1e8e3e', '#ffffff'),
        _ROLE_COLORS['error']: ('#d93025', '#ffffff'),
        _ROLE_COLORS['muted']: ('#dfe3e6', '#3c4043'),
    }
    _ROLE_COLOR_RE = re.compile(r'color:\s*(#[0-9a-fA-F]{6})')

    @staticmethod
    def _make_big_label(point_size, bold=True):
        """状態表示(拡大)タブ用の大きなQLabel(PC画面から離れた位置からでも
        読める文字サイズにするため、統合操作タブの通常ラベルとは別にここで
        フォントを指定する)。"""
        label = QLabel()
        label.setWordWrap(True)
        font = QFont()
        font.setPointSize(point_size)
        font.setBold(bold)
        label.setFont(font)
        return label

    def _build_status_display_tab(self, parent):
        """PC画面から離れた位置(遠く)からでも読めることを想定した表示専用タブ。
        操作ボタンは置かず、選択中ワークと試合中に確認したい主要ステータス
        (動作モード・ポンプ・ピック/投入シーケンス状態・現在状態)を大きく表示
        するだけにする(値の実体は統合操作タブ側のラベル・ワークボタンのまま、
        二重管理を避けるため_refresh_status_display_tabで都度ミラーする)。
        選択中ワークは、フィールド上の実際のワーク配置(_build_field_buttons/
        _build_button_grid)と対応づけたグリッド(X昇順=左->右、Y降順=奥->手前)
        で表示し、選択中セルだけを目立つ色で塗る(ボタンではなく表示専用の
        QLabelにして誤操作を防ぐ)。"""
        layout = QVBoxLayout(parent)

        work_box = QGroupBox('選択中ワーク (フィールド配置)')
        work_outer = QVBoxLayout(work_box)
        axis_top = QLabel('← X- ・ X+ →')
        axis_top.setAlignment(Qt.AlignCenter)
        work_outer.addWidget(axis_top)

        work_grid_layout = QGridLayout()
        work_grid_layout.setSpacing(6)
        self._status_work_cells = {}
        for rc, (x, y, _z) in self._work_grid.items():
            rank, col = rc
            btn = self._work_buttons.get((round(x, 6), round(y, 6)))
            label_text = btn.text() if btn is not None else '?'
            cell = self._make_big_label(22)
            cell.setAlignment(Qt.AlignCenter)
            cell.setMinimumSize(110, 80)
            cell.setText(label_text)
            cell.setProperty('_col_label', label_text.rsplit('-', 1)[-1])
            work_grid_layout.addWidget(cell, rank, col)
            self._status_work_cells[rc] = cell
        work_outer.addLayout(work_grid_layout)

        axis_bottom = QLabel('↑ Y+ (ワーク側) ／ Y- (機体側) ↓')
        axis_bottom.setAlignment(Qt.AlignCenter)
        work_outer.addWidget(axis_bottom)
        layout.addWidget(work_box)

        mode_box = QGroupBox('動作モード')
        mode_layout = QVBoxLayout(mode_box)
        self.big_mode_label = self._make_big_label(40)
        self.big_mode_label.setAlignment(Qt.AlignCenter)
        mode_layout.addWidget(self.big_mode_label)
        layout.addWidget(mode_box)

        pump_box = QGroupBox('ポンプ')
        pump_layout = QVBoxLayout(pump_box)
        self.big_pump_label = self._make_big_label(40)
        self.big_pump_label.setAlignment(Qt.AlignCenter)
        pump_layout.addWidget(self.big_pump_label)
        layout.addWidget(pump_box)

        sequence_box = QGroupBox('ピック/投入シーケンス')
        sequence_layout = QVBoxLayout(sequence_box)
        self.big_sequence_label = self._make_big_label(28)
        self.big_sequence_label.setAlignment(Qt.AlignCenter)
        sequence_layout.addWidget(self.big_sequence_label)
        layout.addWidget(sequence_box)

        current_box = QGroupBox('現在状態')
        current_layout = QVBoxLayout(current_box)
        self.big_current_label = self._make_big_label(24, bold=False)
        current_layout.addWidget(self.big_current_label)
        layout.addWidget(current_box)

        layout.addStretch(1)

    def _style_status_work_cell(self, cell, col_label, selected):
        if selected:
            cell.setStyleSheet(
                'background-color:#2ecc71; color:#0b3d24; border:4px solid #1e8449; '
                'border-radius:8px;')
        elif col_label in ('2', '5'):
            # 元のワークボタン(_build_button_grid)と同じく、3個ずつ回収する
            # 運用で押すべき列(列2・列5)をオレンジで目立たせる。
            cell.setStyleSheet(
                'background-color:#f0ad4e; color:white; border:1px solid #d68f2e; '
                'border-radius:8px;')
        else:
            cell.setStyleSheet(
                'background-color:#ecf0f1; color:#2c3e50; border:1px solid #bdc3c7; '
                'border-radius:8px;')

    def _apply_status_block_style(self, target_label, source_label):
        """source_label(統合操作タブ側)の文字色から役割(info/success/error/
        muted)を逆引きし、状態表示(拡大)タブでは文字色だけでなく背景も塗って
        遠目にも分かるカード状にする。"""
        match = self._ROLE_COLOR_RE.search(source_label.styleSheet())
        color = match.group(1) if match else _ROLE_COLORS['muted']
        bg, fg = self._STATUS_BLOCK_STYLES.get(color, self._STATUS_BLOCK_STYLES[_ROLE_COLORS['muted']])
        target_label.setText(source_label.text())
        target_label.setStyleSheet(f'background-color:{bg}; color:{fg}; border-radius:10px; padding:14px;')

    def _refresh_status_display_tab(self):
        for rc, cell in self._status_work_cells.items():
            self._style_status_work_cell(cell, cell.property('_col_label'), rc == self._selected_work_rc)

        self._apply_status_block_style(self.big_mode_label, self.mode_status_label)
        self._apply_status_block_style(self.big_pump_label, self.sequence_pump_status_label)
        self._apply_status_block_style(self.big_sequence_label, self.sequence_status_label)

        self.big_current_label.setText(self.current_label.text())
        self.big_current_label.setStyleSheet(self.current_label.styleSheet())

    def _build_gain_tab(self, parent):
        self._build_panel_tab(
            parent,
            top_funcs=[self._build_apply_all_panel],
            left_funcs=[self._build_trajectory_panel, self._build_joy_speed_panel],
            right_funcs=[self._build_mit_gain_panel, self._build_robomas_gain_panel,
                         self._build_robomas_autotune_panel, self._build_robomas_vel_gain_panel,
                         self._build_velocity_mode_panel])

    def _build_calibration_tab(self, parent):
        self._build_panel_tab(
            parent,
            left_funcs=[self._build_axis_enable_panel, self._build_machine_origin_offset_panel,
                        self._build_hand_offset_panel, self._build_origin_panel,
                        self._build_homing_panel])

    def _build_wiring_tab(self, parent):
        self._build_panel_tab(
            parent,
            top_funcs=[self._build_wiring_bulk_load_panel],
            left_funcs=[self._build_robomas_wiring_panel, self._build_cubemars_wiring_panel,
                        self._build_hand_wiring_panel],
            right_funcs=[self._build_homing_wiring_panel, self._build_can_host_raw_monitor_panel,
                         self._build_limit_switch_wiring_panel])

    def _build_wiring_bulk_load_panel(self, layout):
        # 各配線設定パネルは_build_yaml_wiring_panelで起動時に自動読込済みだが、
        # GUI起動後にyamlファイルを外部エディタ等で書き換えた場合に、各パネル
        # 個別の「読込」を毎回押さずまとめて再読込できるようにするボタン
        # (_build_apply_all_panelの「全ゲイン読み込み」と同じ設計、2026-09-07新規)。
        box = QGroupBox('一括読込')
        box_layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, '下記5パネルすべてのyamlを一括で再読込する。\n'
                          '各パネルは本タブを開いた時点で自動読込済み。', 'muted')
        box_layout.addWidget(desc)

        self.wiring_bulk_load_status_label = QLabel()
        self.wiring_bulk_load_status_label.setWordWrap(True)
        _set_status(self.wiring_bulk_load_status_label, '未読込', 'muted')
        box_layout.addWidget(self.wiring_bulk_load_status_label)

        btn = QPushButton('全配線設定 一括読込')
        btn.clicked.connect(self._on_bulk_load_yaml_wiring)
        box_layout.addWidget(btn)

        layout.addWidget(box)

    def _on_bulk_load_yaml_wiring(self):
        failed = []
        for attr_prefix, field_specs, yaml_filename, title in getattr(self, '_yaml_wiring_panels', []):
            if not self._on_load_yaml_wiring(attr_prefix, field_specs, yaml_filename):
                failed.append(title)
        if failed:
            _set_status(self.wiring_bulk_load_status_label,
                        f'{"・".join(failed)}の読込に失敗しました', 'error')
        else:
            _set_status(self.wiring_bulk_load_status_label, '全パネルの読込完了', 'success')

    def _build_apply_all_panel(self, layout):
        # 各ゲインパネル個別の「適用」を毎回押す代わりに、GUIが保持している
        # 軌道生成・MIT・robomas・joy速度の全ゲインを一括で実機へ送る「実機接続」
        # ボタン。読込を経由しなくても、GUI側の値(前回適用/読込時にgains.jsonへ
        # 保存された値、またはその場での編集値)をそのまま送信する。
        box = QGroupBox('実機接続 (GUI保持ゲインを一括適用)')
        box_layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, 'GUIが保持している軌道生成・MIT・robomas・joy速度の\n'
                          '全ゲインを、個別の「適用」なしでまとめて実機へ送信する。', 'muted')
        box_layout.addWidget(desc)

        self.load_all_status_label = QLabel()
        self.load_all_status_label.setWordWrap(True)
        _set_status(self.load_all_status_label, '未読込', 'muted')
        box_layout.addWidget(self.load_all_status_label)

        load_all_btn = QPushButton('全ゲイン読み込み')
        load_all_btn.clicked.connect(self._on_load_all_gains)
        box_layout.addWidget(load_all_btn)

        self.apply_all_status_label = QLabel()
        self.apply_all_status_label.setWordWrap(True)
        _set_status(self.apply_all_status_label, '未送信', 'muted')
        box_layout.addWidget(self.apply_all_status_label)

        btn = QPushButton('全ゲイン適用')
        btn.setProperty('variant', 'danger')
        btn.clicked.connect(self._on_apply_all_gains)
        box_layout.addWidget(btn)

        layout.addWidget(box)

    def _on_load_all_gains(self):
        # 各パネル個別の「読込」を毎回押す代わりに、実機(trajectory_follower_node/
        # joy_teleop_node)から軌道生成・MIT・robomas・joy速度の全ゲインをまとめて
        # 読み込む。結果は各パネル自身のstatus_labelに反映される(非同期のため)。
        results = {
            'trajectory_follower_node(軌道生成)': self._on_load_traj_params(),
            'trajectory_follower_node(MIT)': self._on_load_mit_gains(),
            'trajectory_follower_node(robomas)': self._on_load_robomas_gains(),
            'trajectory_follower_node(速度モード)': self._on_load_robomas_vel_gains(),
            'joy_teleop_node': self._on_load_joy_speed(),
        }
        failed = [label for label, ok in results.items() if not ok]
        if failed:
            _set_status(self.load_all_status_label,
                        f'{"・".join(failed)}に接続できません(未起動?)', 'error')
        else:
            _set_status(self.load_all_status_label, '各パネルへ読込中...', 'muted')

    def _on_apply_all_gains(self):
        try:
            traj = self._collect_traj_values()
            mit = self._collect_mit_values()
            robomas = self._collect_robomas_values()
            robomas_vel = self._collect_robomas_vel_values()
            joy = self._collect_joy_speed_values()
        except (ValueError, KeyError):
            QMessageBox.critical(
                self, '入力エラー',
                '軌道生成・MIT・robomas・速度モード・joy速度のいずれかに数値以外の入力があります')
            return

        reply = QMessageBox.question(
            self, '全ゲイン一括適用の確認',
            'GUIが保持している軌道生成パラメータ・MITゲイン・robomasゲイン・\n'
            '速度モードゲイン・joy速度を、まとめて実機(trajectory_follower_node/\n'
            'joy_teleop_node)へ即座に反映します。\n\n'
            'Kpを大きくするほど保持力・応答性が上がりますが、\n'
            '実機にかかる力も大きくなります。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        self._persist_traj_values(traj)
        self._persist_mit_values(mit)
        self._persist_gains('robomas_gain', robomas)
        self._persist_gains('robomas_vel_gain', robomas_vel)
        self._persist_gains('joy_speed', joy)

        traj_node_values = dict(traj)
        traj_node_values.update(mit)
        traj_node_values.update(robomas)
        traj_node_values.update(robomas_vel)
        self._apply_all_results = {}
        ok_traj = self.node.set_node_params(TRAJ_NODE_NAME, traj_node_values, self._apply_all_traj_result)
        ok_joy = self.node.set_node_params(JOY_NODE_NAME, joy, self._apply_all_joy_result)
        if ok_traj and ok_joy:
            _set_status(self.apply_all_status_label, '送信中...', 'muted')
        else:
            missing = []
            if not ok_traj:
                missing.append('trajectory_follower_node')
                self._apply_all_results['traj'] = None
            if not ok_joy:
                missing.append('joy_teleop_node')
                self._apply_all_results['joy'] = None
            _set_status(self.apply_all_status_label,
                        f'{"・".join(missing)}に接続できません(未起動?)', 'error')
        # 両方未接続の場合はコールバックが一つも発火しないため、ここで確定させる
        # (片方のみ未接続の場合はもう片方のコールバックが後から発火してfinalizeする)。
        self._finalize_apply_all()

    def _apply_all_traj_result(self, results):
        self._apply_all_results['traj'] = results
        self._finalize_apply_all()

    def _apply_all_joy_result(self, results):
        self._apply_all_results['joy'] = results
        self._finalize_apply_all()

    def _finalize_apply_all(self):
        if 'traj' not in self._apply_all_results or 'joy' not in self._apply_all_results:
            return
        problems = []
        for label, results in (
                ('trajectory_follower_node', self._apply_all_results['traj']),
                ('joy_teleop_node', self._apply_all_results['joy'])):
            if results is None:
                problems.append(f'{label}: 応答なし')
            elif not all(r.successful for r in results):
                reasons = '; '.join(r.reason for r in results if not r.successful)
                problems.append(f'{label}: {reasons}')
        if problems:
            _set_status(self.apply_all_status_label, '一部失敗: ' + ' / '.join(problems), 'error')
        else:
            _set_status(self.apply_all_status_label, '全ゲインを適用しました', 'success')

    def _build_machine_status_panel(self, column):
        # 統合操作タブに一目で機体の稼働状況が分かるサマリを置く。ノード起動状況は
        # GUIが直接やり取りする4ノード(STATUS_NODE_NAMES)をget_node_names()で
        # ポーリングして判定する(_refresh_machine_status参照)。適用ゲインは
        # ゲイン調整タブの各パネルが既に持つstatus_label(読込/適用結果)の文言・色を
        # そのままミラーするだけで、値の二重管理を避けている。
        box = QGroupBox('機体ステータス')
        layout = QVBoxLayout(box)

        layout.addWidget(QLabel('ノード起動状況'))
        node_grid = QGridLayout()
        self.node_status_labels = {}
        for i, name in enumerate(STATUS_NODE_NAMES):
            node_grid.addWidget(QLabel(name), i, 0)
            label = QLabel()
            _set_status(label, '確認中...', 'muted')
            self.node_status_labels[name] = label
            node_grid.addWidget(label, i, 1)
        layout.addLayout(node_grid)

        layout.addWidget(QLabel('適用ゲイン'))
        gain_grid = QGridLayout()
        self.gain_summary_labels = {}
        for i, (key, title) in enumerate((
                ('traj', '軌道生成'), ('mit', 'MIT'), ('robomas', 'robomas'), ('joy', 'joy速度'))):
            gain_grid.addWidget(QLabel(title), i, 0)
            label = QLabel()
            label.setWordWrap(True)
            _set_status(label, '未読込', 'muted')
            self.gain_summary_labels[key] = label
            gain_grid.addWidget(label, i, 1)
        layout.addLayout(gain_grid)

        layout.addWidget(QLabel('校正状態'))
        calib_grid = QGridLayout()
        self.calibration_summary_labels = {}
        for i, (key, title) in enumerate((
                ('machine_origin', '機体原点オフセット'), ('root_theta', 'root_theta原点'),
                ('homing', 'z/rホーミング'))):
            calib_grid.addWidget(QLabel(title), i, 0)
            label = QLabel()
            label.setWordWrap(True)
            _set_status(label, '未実行', 'muted')
            self.calibration_summary_labels[key] = label
            calib_grid.addWidget(label, i, 1)
        layout.addLayout(calib_grid)

        column.addWidget(box)

    def _refresh_machine_status(self):
        self._update_status_leds()
        self._refresh_autotune_progress()
        active = self.node.get_active_node_names()
        for name, label in self.node_status_labels.items():
            if name in active:
                _set_status(label, '起動中', 'success')
            else:
                _set_status(label, '未起動', 'error')

        for key, source_label in (
                ('traj', self.traj_status_label), ('mit', self.mit_gain_status_label),
                ('robomas', self.robomas_gain_status_label), ('joy', self.joy_speed_status_label)):
            target = self.gain_summary_labels[key]
            target.setText(source_label.text())
            target.setStyleSheet(source_label.styleSheet())

        for key, source_label in (
                ('machine_origin', self.machine_origin_status_label),
                ('root_theta', self.origin_status_label),
                ('homing', self.homing_status_label)):
            target = self.calibration_summary_labels[key]
            target.setText(source_label.text())
            target.setStyleSheet(source_label.styleSheet())

        if self._launch_process is not None:
            code = self._launch_process.poll()
            if code is None:
                _set_status(self.launch_status_label, '起動中', 'success')
            elif code == 0:
                _set_status(self.launch_status_label, '終了しました', 'muted')
            else:
                _set_status(self.launch_status_label, f'終了しました(code={code})', 'error')

    def _build_setup_checklist_panel(self, column):
        # 試合開始前に毎回行う一連の操作(全ノード起動・全ゲイン適用・原点校正)を
        # 統合操作タブから離れずに実行できるようにする実行ボタン群。各ボタンは
        # ゲイン調整/原点校正タブの既存ハンドラをそのまま呼ぶだけで、値の入力欄自体は
        # 元のタブに残す(二重管理を避ける)。実行結果は機体ステータスパネル
        # (適用ゲイン・校正状態)にミラー表示される。
        box = QGroupBox('実機セットアップ')
        layout = QVBoxLayout(box)

        warn = QLabel()
        warn.setWordWrap(True)
        _set_status(warn, '(事前確認) 緊急停止ボタンを押すこと', 'error')
        layout.addWidget(warn)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, '上から順に実行する想定(ノード起動→ゲイン適用→原点校正)。\n'
                          '各操作の詳細な値編集はゲイン調整・原点校正タブで行う。\n'
                          '進捗は左の機体ステータスパネル(適用ゲイン・校正状態)で確認できる。', 'muted')
        layout.addWidget(desc)

        # rvizは別途起動済みの場合に二重起動を避けたいことがあるため、「全ノード
        # 起動」のたびにON/OFFできるようにする。ros2canは常に起動する(実機接続の
        # 前提として必須)が、GUI(PyQt5ウィンドウ)か--nogui(ターミナルダッシュ
        # ボード)かは選べるようにする(2026-09-07追加、ユーザー指摘: 「全ノード
        # 起動ボタンで起動するrvizとros2canの起動を管理できるチェックボックスを
        # 追加」→「ros2canは起動するよもちろん。チェックボックスで選ぶのは
        # ros2can --noguiか否かだけ」)。既定はrviz起動ON・ros2can GUI(従来通り)。
        launch_options_row = QHBoxLayout()
        self.launch_use_viz_checkbox = QCheckBox('rvizを起動する')
        self.launch_use_viz_checkbox.setChecked(True)
        self.launch_ros2can_gui_checkbox = QCheckBox('ros2canをGUIで起動する(オフで--nogui)')
        self.launch_ros2can_gui_checkbox.setChecked(True)
        launch_options_row.addWidget(self.launch_use_viz_checkbox)
        launch_options_row.addWidget(self.launch_ros2can_gui_checkbox)
        layout.addLayout(launch_options_row)

        launch_row = QHBoxLayout()
        launch_btn = QPushButton('全ノード起動')
        launch_btn.setProperty('variant', 'primary')  # よく使う操作のため目立つ色に(2026-09-03)
        stop_launch_btn = QPushButton('停止')
        launch_btn.clicked.connect(self._on_launch_all_nodes)
        stop_launch_btn.clicked.connect(self._on_stop_all_nodes)
        launch_row.addWidget(launch_btn)
        launch_row.addWidget(stop_launch_btn)
        layout.addLayout(launch_row)
        self.launch_status_label = QLabel()
        self.launch_status_label.setWordWrap(True)
        _set_status(self.launch_status_label, '未起動', 'muted')
        layout.addWidget(self.launch_status_label)

        gain_row = QHBoxLayout()
        load_gain_btn = QPushButton('全ゲイン読込')
        apply_gain_btn = QPushButton('全ゲイン適用')
        apply_gain_btn.setProperty('variant', 'danger')
        load_gain_btn.clicked.connect(self._on_load_all_gains)
        apply_gain_btn.clicked.connect(self._on_apply_all_gains)
        gain_row.addWidget(load_gain_btn)
        gain_row.addWidget(apply_gain_btn)
        layout.addLayout(gain_row)

        origin_row = QHBoxLayout()
        machine_origin_btn = QPushButton('機体原点オフセット適用')
        root_theta_btn = QPushButton('root_theta原点設定')
        root_theta_btn.setProperty('variant', 'danger')
        machine_origin_btn.clicked.connect(self._on_apply_machine_origin)
        root_theta_btn.clicked.connect(self._on_set_root_theta_origin)
        origin_row.addWidget(machine_origin_btn)
        origin_row.addWidget(root_theta_btn)
        layout.addLayout(origin_row)

        homing_row = QHBoxLayout()
        homing_start_z_btn = QPushButton('z軸ホーミング開始')
        homing_start_z_btn.setProperty('variant', 'primary')
        homing_start_r_btn = QPushButton('r軸ホーミング開始')
        homing_start_r_btn.setProperty('variant', 'primary')
        homing_stop_btn = QPushButton('中断')
        homing_skip_btn = QPushButton('スキップ')
        homing_skip_btn.setProperty('variant', 'danger')
        homing_start_z_btn.clicked.connect(self._on_start_homing_z)
        homing_start_r_btn.clicked.connect(self._on_start_homing_r)
        homing_stop_btn.clicked.connect(self._on_stop_homing)
        homing_skip_btn.clicked.connect(self._on_skip_homing)
        homing_row.addWidget(homing_start_z_btn)
        homing_row.addWidget(homing_start_r_btn)
        homing_row.addWidget(homing_stop_btn)
        homing_row.addWidget(homing_skip_btn)
        layout.addLayout(homing_row)

        column.addWidget(box)

    def _on_launch_all_nodes(self):
        if self._launch_process is not None and self._launch_process.poll() is None:
            QMessageBox.information(self, '起動済み', '既に起動中です(先に停止してください)')
            return
        use_viz = 'true' if self.launch_use_viz_checkbox.isChecked() else 'false'
        # チェックボックスは「GUIで起動する」なので、ros2can_nogui引数へは反転して渡す。
        ros2can_nogui = 'false' if self.launch_ros2can_gui_checkbox.isChecked() else 'true'
        cmd = ALL_AXES_LAUNCH_BASE_CMD + [f'use_viz:={use_viz}', f'ros2can_nogui:={ros2can_nogui}']
        try:
            # start_new_session=True(setsid)でこの子プロセスを独立したプロセス
            # グループのリーダーにする。ros2 launchはさらに複数のノードを自分の
            # 子プロセスとして起動するため、後で停止する際はこのグループ全体へ
            # まとめてシグナルを送る(_signal_launch_process_group参照。
            # 2026-09-03、ユーザー報告: 「停止ボタンが機能しないことがある。
            # 停止完了と表示されても裏で生きていたり、閉じるボタンでも裏で
            # 生きていたりする」ことへの対処)。
            self._launch_process = subprocess.Popen(cmd, start_new_session=True)
        except OSError as exc:
            _set_status(self.launch_status_label, f'起動失敗: {exc}', 'error')
            return
        _set_status(self.launch_status_label, '起動処理中...', 'muted')

    def _signal_launch_process_group(self, sig):
        """self._launch_process(ros2 launch)とその配下の全ノードプロセスへ
        まとめてシグナルを送る。send_signal()はPopenが直接追跡している1プロセス
        (ros2 launch自身)にしか届かず、配下で起動された各ノードのプロセスまでは
        必ずしも終了しきらないことがある(2026-09-03、ユーザー報告: 「停止
        ボタン/閉じるボタンを押しても裏でノードが生き続け、ターミナルで
        Ctrl+Cを押すまで終了しない」)。ターミナルでCtrl+Cを押した場合は
        フォアグラウンドのプロセスグループ全体にSIGINTが届くため確実に
        終了するのに対し、Popen.send_signal()は単一PIDにしか届かないことが
        この差の原因と考えられる。_on_launch_all_nodesでstart_new_session=True
        (setsid)で独立したプロセスグループにしてあるため、os.killpg()で
        グループ全体(ros2 launch本体+配下の全ノード)へ確実に届かせる。"""
        if self._launch_process is None:
            return
        try:
            os.killpg(os.getpgid(self._launch_process.pid), sig)
        except ProcessLookupError:
            pass  # 既に終了済み

    def _on_stop_all_nodes(self):
        if self._launch_process is None or self._launch_process.poll() is not None:
            _set_status(self.launch_status_label, '起動していません', 'muted')
            return
        reply = QMessageBox.question(
            self, '全ノード停止の確認',
            '起動中のノード群を停止します(ros2 launchへSIGINTを送信し、\n'
            '通常のCtrl+Cと同様に各ノードを終了させます)。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self._signal_launch_process_group(signal.SIGINT)
        _set_status(self.launch_status_label, '停止処理中...', 'muted')

    def _build_current_state_panel(self, column):
        box = QGroupBox('現在状態 (リアルタイム)')
        layout = QVBoxLayout(box)
        self.current_label = QLabel()
        self.current_label.setWordWrap(True)
        _set_status(self.current_label, '(mixed_joint_states待ち)', 'success')
        layout.addWidget(self.current_label)

        # フィールド俯瞰ミニマップ+Z軸バーゲージ(2026-09-07新規、ユーザー要望:
        # 「統合操作画面に簡易的かつ視覚的に機体の座標データが分かるような機能」)。
        # 数値(current_label)だけでは位置感を掴みにくいため、XY位置とZ高さを
        # ひと目で見られるようにする。いずれも読み取り専用(_refresh_current_state
        # から更新するのみ、クリック操作は無い)。
        visual_row = QHBoxLayout()
        self.field_minimap = FieldMinimapWidget()
        visual_row.addWidget(self.field_minimap)
        self.z_gauge = ZGaugeWidget(height=self.field_minimap.height())
        visual_row.addWidget(self.z_gauge)
        visual_row.addStretch(1)
        layout.addLayout(visual_row)

        column.addWidget(box)

    def _build_mode_panel(self, column):
        box = QGroupBox('動作モード (trajectory_follower_node)')
        layout = QVBoxLayout(box)
        self._mode_group = QButtonGroup(self)
        for value, label in (('auto', '自動専用 (GUI)'), ('manual', '手動専用 (joy)'), ('both', '併用')):
            rb = QRadioButton(label)
            if value == 'both':
                rb.setChecked(True)
            self._mode_group.addButton(rb)
            self._mode_buttons[value] = rb
            rb.toggled.connect(lambda checked, v=value: checked and self._on_mode_changed(v))
            layout.addWidget(rb)
        self.mode_status_label = QLabel()
        self.mode_status_label.setWordWrap(True)
        _set_status(self.mode_status_label, '未読込', 'muted')
        layout.addWidget(self.mode_status_label)
        column.addWidget(box)

    def _set_mode_silent(self, mode):
        rb = self._mode_buttons.get(mode)
        if rb is None:
            return
        for b in self._mode_buttons.values():
            b.blockSignals(True)
        rb.setChecked(True)
        for b in self._mode_buttons.values():
            b.blockSignals(False)

    def _build_hand_panel(self, column):
        # ハンド(吸着パッド展開/収集サーボ・ワークピッチサーボ・ダイヤフラム
        # ポンプ)の操作パネル。配線・角度・デューティの編集は配線設定タブの
        # 「ハンド配線設定」で行い、ここでは6つのサービス(hand_node)を
        # 呼ぶだけにする(定型位置移動ボタンと同様、確認ダイアログは付けない。
        # 展開/収納・ポンプON/OFF・ピッチ切替は試合中に繰り返し使う通常操作の
        # ため)。展開/収集サーボとポンプは独立している(2026-09-03、ユーザー
        # 指摘で分離: 収納はワークを保持したまま中央へ集める動作のため、
        # 収納時に自動でポンプを切ってはいけない)。
        box = QGroupBox('ハンド (hand_node)')
        layout = QVBoxLayout(box)

        self.hand_status_label = QLabel()
        self.hand_status_label.setWordWrap(True)
        _set_status(self.hand_status_label, '未実行', 'muted')
        layout.addWidget(self.hand_status_label)

        pad_row = QHBoxLayout()
        spread_btn = QPushButton('吸着パッド展開')
        gather_btn = QPushButton('吸着パッド収納')
        spread_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_spread_pads'))
        gather_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_gather_pads'))
        pad_row.addWidget(spread_btn)
        pad_row.addWidget(gather_btn)
        layout.addLayout(pad_row)

        pump_row = QHBoxLayout()
        pump_on_btn = QPushButton('ポンプON(吸着)')
        pump_on_btn.setProperty('variant', 'primary')
        pump_off_btn = QPushButton('ポンプOFF(真空破壊)')
        pump_on_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_pump_on'))
        pump_off_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_pump_off'))
        pump_row.addWidget(pump_on_btn)
        pump_row.addWidget(pump_off_btn)
        layout.addLayout(pump_row)

        pitch_row = QHBoxLayout()
        hold_btn = QPushButton('ピッチ: 保持姿勢')
        insert_btn = QPushButton('ピッチ: 投入姿勢')
        hold_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_set_pitch_hold'))
        insert_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_set_pitch_insert'))
        pitch_row.addWidget(hold_btn)
        pitch_row.addWidget(insert_btn)
        layout.addLayout(pitch_row)

        column.addWidget(box)

    def _on_hand_trigger(self, service_name):
        _set_status(self.hand_status_label, f'{service_name} 呼び出し中...', 'muted')
        ok = self.node.call_trigger_service(
            service_name, functools.partial(self._on_hand_trigger_done, service_name))
        if not ok:
            _set_status(self.hand_status_label, 'hand_nodeに接続できません(未起動?)', 'error')
            # 自動シーケンス側がこのサービスの応答待ちだった場合、応答が永久に
            # 来ないままシーケンスが止まり続けるのを防ぐため中断する。
            if self._seq_active and self._seq_waiting_service == service_name:
                self._abort_sequence(
                    f'{self._seq_kind}: {service_name} に接続できません(hand_node未起動?)')

    def _on_hand_trigger_done(self, service_name, success, message):
        _set_status(self.hand_status_label, message, 'success' if success else 'error')
        # ピック/投入シーケンスが'call'ステップとしてこのサービスを自ら呼んでいた
        # 場合(_advance_hand_step参照。'wait_pick_confirm'は/pick_sequence_confirm
        # サービス経由で別途判定するため、ここは通らない)、次のステップへ進める。
        # response.success=Falseはhand_node側の設計上「device_id未配線でCAN送信を
        # スキップした」ことを意味するだけで、RViz表示用のpublishはその場合でも
        # 既に行われている(hand_node.py _set_deploy/_set_pitch参照)。配線前でも
        # シーケンスをRVizのみで最後まで確認できるよう、これはシーケンスを中断せず
        # 警告表示のみに留める(サービス自体に到達できない場合は_on_hand_triggerの
        # ok=False側で中断する)。
        if self._seq_active and self._seq_waiting_service == service_name:
            self._seq_waiting_service = None
            self._seq_index += 1
            if not success:
                _set_status(self.sequence_status_label,
                            f'{self._seq_kind}: {service_name} 警告(配線未設定?): {message}', 'error')

    # ---------- ピック/投入 自動シーケンス ----------
    def _build_sequence_settings_panel(self, column):
        # ワーク・シューティングボックスパネル(_build_field_buttons)の隣に置く
        # 設定・状態パネル(2026-09-03、ユーザー指定でこの位置に変更)。
        # 「自動シーケンスで実行」チェックボックスと連動する。各数値はGUIの
        # QLineEditが真値で、「適用」はgains.jsonへの永続化のみ行う(他のゲイン系
        # パネルと違いリモートノードへのSetParametersは無い。シーケンス開始時に
        # このGUIプロセス自身が_collect_sequence_valuesで直接読み出すため)。
        box = QGroupBox('ピック/投入 自動シーケンス')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(
            desc,
            '「ワーク・シューティングボックス」パネルの「自動シーケンスで実行」を\n'
            'ONにしてボタンを押すと使える。\n'
            '回収: ハンドを保持姿勢・パッド展開・ポンプONにしてからワーク手前\n'
            '(アプローチ高さ)まで自動で近づき、そこから先(実際の接触・回収)は\n'
            '下の「回収実行」ボタンまたはPSコンのPSボタンを押すまで進まない\n'
            '(×ボタンは選択中ワークへの移動のみ。矢印キーでワークを選択できる)。\n'
            '投入: シューティングエリア(L4/R4固定)の真上に安全高度を維持したまま\n'
            '自動で近づくだけで、シュート自体(降下・位置調整・ポンプOFF)は行わない。\n'
            '以降はZ軸の降下も含めて全てコントローラーで手動操作する。下の\n'
            '「L4へ移動」「R4へ移動」ボタン(またはPSコンの割当ボタン)でいつでも\n'
            'この移動だけをやり直せる。', 'muted')
        layout.addWidget(desc)

        self.sequence_status_label = QLabel()
        self.sequence_status_label.setWordWrap(True)
        _set_status(self.sequence_status_label, '待機中', 'muted')
        layout.addWidget(self.sequence_status_label)

        btn_row = QHBoxLayout()
        pick_confirm_btn = QPushButton('回収実行')
        pick_confirm_btn.setProperty('variant', 'primary')
        pick_confirm_btn.clicked.connect(self._on_pick_confirm_button_clicked)
        btn_row.addWidget(pick_confirm_btn)
        shoot_l4_btn = QPushButton('L4へ移動')
        shoot_l4_btn.clicked.connect(lambda: self._on_shoot_start_requested('L4'))
        btn_row.addWidget(shoot_l4_btn)
        shoot_r4_btn = QPushButton('R4へ移動')
        shoot_r4_btn.clicked.connect(lambda: self._on_shoot_start_requested('R4'))
        btn_row.addWidget(shoot_r4_btn)
        abort_btn = QPushButton('中断')
        abort_btn.setProperty('variant', 'danger')
        abort_btn.clicked.connect(self._on_abort_sequence)
        btn_row.addWidget(abort_btn)
        layout.addLayout(btn_row)

        # ポンプON/OFFはハンドパネルと重複するが、シーケンス操作中にタブを
        # 切り替えずに吸着のON/OFFができるよう、ここにも同じ操作を置く
        # (2026-09-03、ユーザー指定: 「ポンプのオンオフボタンをピック投入
        # シーケンスの欄にも配置。ポンプのステータスも見れるように」)。
        pump_row = QHBoxLayout()
        seq_pump_on_btn = QPushButton('ポンプON(吸着)')
        seq_pump_on_btn.setProperty('variant', 'primary')
        seq_pump_on_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_pump_on'))
        pump_row.addWidget(seq_pump_on_btn)
        seq_pump_off_btn = QPushButton('ポンプOFF(真空破壊)')
        seq_pump_off_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_pump_off'))
        pump_row.addWidget(seq_pump_off_btn)
        layout.addLayout(pump_row)

        self.sequence_pump_status_label = QLabel()
        self.sequence_pump_status_label.setWordWrap(True)
        _set_status(self.sequence_pump_status_label, 'ポンプ: 不明', 'muted')
        layout.addWidget(self.sequence_pump_status_label)

        grid = QGridLayout()
        self.sequence_edits = {}
        for i, (key, label) in enumerate((
                ('safe_transit_z_m', '安全高度Z[m]'),
                ('r_retract_m', 'R退避量[m]'),
                ('pickup_z_offset_m', '回収Zオフセット[m]'),
                ('pickup_approach_clearance_m', '回収アプローチ余裕[m]'),
                ('shoot_tip_theta_rad', '投入時手先θ[rad](暫定)'),
                ('gather_settle_sec', '収納後の待ち時間[s]'))):
            grid.addWidget(QLabel(label), i, 0)
            edit = make_float_edit(DEFAULT_SEQUENCE_SETTINGS[key])
            self.sequence_edits[key] = edit
            grid.addWidget(edit, i, 1)
        layout.addLayout(grid)

        apply_btn = QPushButton('適用(値を保存)')
        apply_btn.clicked.connect(self._on_apply_sequence_settings)
        layout.addWidget(apply_btn)

        column.addWidget(box)

    def _collect_sequence_values(self):
        return {key: get_float(edit) for key, edit in self.sequence_edits.items()}

    def _on_apply_sequence_settings(self):
        try:
            values = self._collect_sequence_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'シーケンス設定パネルの各欄に数値を入力してください')
            return
        self._persist_gains('sequence', values)
        _set_status(self.sequence_status_label, '設定を保存しました', 'success')

    def _start_pick_sequence(self, x, y, z):
        """workボタン用: ピッチ保持姿勢・パッド展開・ポンプONを移動開始前に
        まとめて行ってから、安全高度退避→R退避→θ回転→R伸長→ワーク手前
        (アプローチ高さ)まで自動降下→(人間の「回収実行」指示待ち。GUIの
        「回収実行」ボタンまたはPSコンの回収確認ボタン)→ワークに当たる高さまで
        降下→上昇→パッド収納→ピッチ投入姿勢、という回収シーケンス
        (2026-09-03、ユーザー指摘: 「半自動化を大雑把にしよう。ワークへの移動を
        指示されたらハンドを保持姿勢、パッド展開、ポンプオンにして向かう。人の
        操作でワークを回収する。回収時のZ軸下降はそのまま残しておいて」。以前は
        ポンプONを最終降下の後(回収実行を押した後)に自動で呼んでいたが、
        移動開始前に前倒しし、実際の接触・吸着(=回収そのもの)は人間の操作に
        委ねる形にした。Z軸の自動降下(アプローチ高さまで→回収実行待ち→
        最終降下)自体は変更せずそのまま維持している)。パッド収納とピッチ切替の
        間はgather_settle_sec秒待ってから行う(ユーザー指摘: 「収納から姿勢変更
        までの待機時間も必要。ほぼ同時はまずい」。パッドが物理的に収納し切る前に
        ピッチが回り始めると干渉する恐れがあるため)。
        theta回転以降は手先θ(tip_theta_joint)も同時に制御し、吸着パッド3個の
        展開軸がワークの行と平行になるよう自動で打ち消す(2026-09-03、ユーザー
        指摘: 「ハンドは3つ一気に回収するので手先θはワークの行と平行になるように
        動く必要がある」)。
        既に回収シーケンス実行中(回収実行待ち含む)に再度workボタンが押された
        場合は、中断を要求せず自動的に新しいワークへやり直す(2026-09-03、
        ユーザー指摘: 「ワークに移動し回収実行待ちの時に再度ワークの位置が
        送信された場合は自動的にワーク移動からやり直すように」)。この場合は
        直前のワークから近距離の移動とみなし、R軸を旋回軸まで戻す退避を省略して
        直接回転+伸縮する(ユーザー指摘: 「ワークからワークに移動する際はR軸を
        戻さなくていい。近距離なので」)。
        ピッチ保持姿勢・パッド展開・ポンプONの呼び出しはポンプの現在ON/OFF状態に
        関わらず常に行う(2026-09-03、ユーザー指摘: 「ポンプのオンオフによる機体の
        動作制約がないようにしたい」。一時、ポンプが既にONの間はこれらの呼び出しを
        省略する実装を試したが、「回収実行を押してもハンドの向きが現状維持の
        ままになる」という問題が生じたため撤回し、常に呼ぶ方式に戻した)。
        シーケンス完了/中断後で非アクティブな場合でも、この呼び出しは常に
        self._last_pick_targetを更新する。これにより「回収実行」ボタン
        (_on_pick_confirm_requested)は、シーケンスが完了した後でも同じワークへの
        回収シーケンスを再度呼び出せる(吸着失敗時の再試行用、2026-09-03、
        ユーザー指摘: 「吸着できなかったときに回収実行を再度行うことがある。
        現状だと回収実行が一回しかできない」)。
        他のシーケンス(投入シーケンス含む)実行中でも、確認や中断操作なしに
        即座にこちらへ切り替える(2026-09-03、ユーザー指摘:「回収実行を押さ
        なくてもシューティング位置へ移動できるように。ユーザーの動きを制限
        したくない。状況判断はユーザーが行うため」。以前は投入シーケンスなど
        「回収」以外が実行中だとQMessageBoxで拒否していたが、状況判断は人間
        (ユーザー)に委ね、システム側では止めない方針に変更した)。"""
        restarting_from_work = self._seq_active and self._seq_kind == '回収'
        if self._seq_active:
            self._abort_sequence(f'{self._seq_kind}シーケンスを中断し、回収シーケンスへ切り替えます')
        if not self.node.has_current_state():
            QMessageBox.information(self, '未取得', 'まだ現在位置を受信していません')
            return
        try:
            settings = self._collect_sequence_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'シーケンス設定パネルの数値を確認してください')
            return
        self._last_pick_target = (x, y, z)

        pos = self.node.get_current_positions()
        theta0, r0 = pos['root_theta_joint'], pos['r_joint']
        target_theta, target_r = _theta_r_from_xy(x, y)
        safe_zj = clamp(settings['safe_transit_z_m'] - Z_OFFSET, Z_LOWER, Z_UPPER)
        final_zj = clamp((z + settings['pickup_z_offset_m']) - Z_OFFSET, Z_LOWER, Z_UPPER)
        approach_zj = clamp(
            (z + settings['pickup_z_offset_m'] + settings['pickup_approach_clearance_m']) - Z_OFFSET,
            Z_LOWER, Z_UPPER)
        r_retract = clamp(settings['r_retract_m'], R_LOWER, R_UPPER)
        # 吸着パッド3個の展開軸をワークの行(ワールドX軸)と平行に保つため、
        # 手先θ(tip_theta_joint)をroot_thetaと逆方向に同じ量だけ回して打ち消す
        # (SHOOT_TIP_THETA_RAD付近のコメント参照、2026-09-03ユーザー指摘)。
        # theta回転を始めるレグ(パッドが目的の行へ向く前)から適用する。
        tip_theta_pick = -target_theta

        if restarting_from_work:
            # ワークtoワークの近距離移動: R退避を省略し、回転とR伸縮を1レグで行う。
            approach_legs = [('move', target_theta, safe_zj, target_r, tip_theta_pick)]
        else:
            approach_legs = [
                ('move', theta0, safe_zj, r_retract),         # Rを旋回軸近くへ退避
                ('move', target_theta, safe_zj, r_retract, tip_theta_pick),  # theta回転(手先θも同時に打ち消し)
                ('move', target_theta, safe_zj, target_r, tip_theta_pick),   # Rを目標半径まで伸長
            ]

        steps = [
            ('call', '/hand_set_pitch_hold'),             # 回収待機中は保持姿勢にしておく
            ('call', '/hand_spread_pads'),                # 移動前にパッド展開
            ('call', '/hand_pump_on'),                    # 移動前に吸着ON(接触したらすぐ吸着できるように)
            ('move', theta0, safe_zj, r0),               # 現在地のまま安全高度へ
            *approach_legs,
            ('move', target_theta, approach_zj, target_r, tip_theta_pick),  # ワーク手前まで自動降下
            ('wait_pick_confirm',),                       # 人間の「回収実行」指示待ち(=人の操作でワークを回収)
            ('move', target_theta, final_zj, target_r, tip_theta_pick),  # ワークに当たる高さまで降下
            ('move', target_theta, safe_zj, target_r, tip_theta_pick),  # 安全高度へ上昇
            ('call', '/hand_gather_pads'),
            ('delay', settings['gather_settle_sec']),     # パッドが物理的に収納し切るまで少し待つ
            ('call', '/hand_set_pitch_insert'),           # 収納後は投入姿勢へ(シュートへの搬送に備える)
        ]
        self._begin_sequence('回収', steps)

    def _start_shoot_sequence(self, x, y, z):
        """shootボタン用: 安全高度を維持したままシューティングエリアのXYへ
        向かうだけの投入シーケンス(2026-09-03、ユーザー指摘: 「半自動化を
        大雑把にしよう。特にシューティングについてはシューティングエリアに
        安全高度を維持したまま向かうだけでシュート自体は行わない」)。Z軸の
        降下・ポンプOFFはいずれも自動区間に含めず、すべて人間がコントローラーで
        操作する(ユーザー指摘: 「シューティングエリアへ移動人が位置を調整し
        ポンプをオフする。Z軸の操作も人が行う」)。手先ピッチを投入姿勢へ
        揃えることだけは例外で、シーケンス開始時に自動で行う(下記steps先頭の
        hand_set_pitch_insert呼び出し、2026-09-04追加。当初は回収シーケンス末尾
        での自動切替に任せて投入シーケンス側では何もしていなかったが、
        「回収実行を押さなくてもシューティング位置へ移動できるように」により
        回収シーケンスをいつでも中断できるようになった結果、保持姿勢のまま
        シュートへ向かってしまうことがあり「手先ピッチが作動したりしなかったり
        する」不具合として顕在化したため、投入シーケンス自身が保証するように
        変更した。ユーザー指摘: 「シュート時は手先ピッチ投入姿勢でなくては
        ならない」)。
        theta回転以降は手先θ(tip_theta_joint)もr方向に垂直な一定値
        (shoot_tip_theta_rad)へ制御する(2026-09-03、ユーザー指摘: 「シュート時は
        Rと垂直になるように」。この値自体は実機未検証の暫定値)。
        実運用ではx, y, zはSHOOT_FIXED_TARGETS(L4/R4)固定で、「L4へ移動」
        「R4へ移動」ボタン(_on_shoot_start_requested、GUIボタンまたはPSコンの
        割当ボタン)経由で呼ばれる想定(2026-09-03、ユーザー指摘: 「シューティング
        エリアはL4もしくはR4で、ボタンを2つおいておいて」)。
        他のシーケンス(回収シーケンス含む、回収実行待ちの状態でも)実行中でも、
        確認や中断操作なしに即座にこちらへ切り替える(2026-09-03、ユーザー指摘:
        「回収実行を押さなくてもシューティング位置へ移動できるように。ユーザー
        の動きを制限したくない。状況判断はユーザーが行うため」。以前は
        _guard_sequence_startでQMessageBoxにより拒否していたが、状況判断は
        人間(ユーザー)に委ね、システム側では止めない方針に変更した。これに
        伴い、実行中は開始できないことを前提とした「1件だけ予約して完了後に
        自動開始する」キュー機構(_seq_queued_shoot_label、_on_shoot_start_
        requested参照)も不要になり削除した)。"""
        if self._seq_active:
            self._abort_sequence(f'{self._seq_kind}シーケンスを中断し、投入シーケンスへ切り替えます')
        if not self.node.has_current_state():
            QMessageBox.information(self, '未取得', 'まだ現在位置を受信していません')
            return
        try:
            settings = self._collect_sequence_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'シーケンス設定パネルの数値を確認してください')
            return

        # 投入シーケンスは手先θを固定値(tip_theta_shoot、下記)へ制御するため、
        # joy_teleop_node側の手先θroot_theta追従(OPTIONSボタン、既定ON)が
        # ONのままだと毎周期-root_thetaへ上書きされて競合する。シーケンス開始時に
        # 自動でOFFにする(2026-09-03、ユーザー指定:「手先θ追従はシューティング
        # ボックスへの自動移動時には自動で無効化」。joy_teleop_node未起動時は
        # set_joy_tip_theta_follow内で黙って無視されるだけで、本シーケンス自体は
        # 続行する)。
        self.node.set_joy_tip_theta_follow(False)

        pos = self.node.get_current_positions()
        theta0, r0 = pos['root_theta_joint'], pos['r_joint']
        target_theta, target_r = _theta_r_from_xy(x, y)
        safe_zj = clamp(settings['safe_transit_z_m'] - Z_OFFSET, Z_LOWER, Z_UPPER)
        r_retract = clamp(settings['r_retract_m'], R_LOWER, R_UPPER)
        # 投入時はr方向に垂直な姿勢(root_thetaの値によらず一定のtip_theta)にする
        # (SHOOT_TIP_THETA_RAD付近のコメント参照、ユーザー指摘:「シュート時はRと
        # 垂直になるように」。値自体は未検証の暫定値)。
        tip_theta_shoot = settings['shoot_tip_theta_rad']

        steps = [
            # シュート時は手先ピッチが投入姿勢でなければならない(2026-09-04、
            # ユーザー指摘: 「手先ピッチが作動したりしなかったりする理由。
            # シュート時は手先ピッチ投入姿勢でなくてはならない」)。以前はこの
            # 呼び出しが無く、回収シーケンス末尾の(call, '/hand_set_pitch_insert')
            # (パッド収納後に投入姿勢へ切り替える箇所)が完了した場合のみ結果的に
            # 投入姿勢になっていた。2026-09-03の「回収実行を押さなくても
            # シューティング位置へ移動できるように」「ユーザーの動きを制限したく
            # ない」により、回収シーケンスを最後まで待たずいつでも即座に投入
            # シーケンスへ中断・切り替えられるようになったため、回収シーケンスが
            # 保持姿勢(hand_set_pitch_hold、シーケンス冒頭)のまま・あるいは
            # ピッチ未設定のまま中断された状態で投入シーケンスが始まるケースが
            # 増え、「手先ピッチが作動したりしなかったりする」不具合として顕在化
            # した。原因は投入シーケンス自身がピッチ姿勢を一切指定していなかった
            # ことなので、シーケンス開始時に必ず投入姿勢へ揃えるようにする。
            ('call', '/hand_set_pitch_insert'),
            ('move', theta0, safe_zj, r0),
            ('move', theta0, safe_zj, r_retract),
            ('move', target_theta, safe_zj, r_retract, tip_theta_shoot),
            ('move', target_theta, safe_zj, target_r, tip_theta_shoot),  # 安全高度のままシューティングエリアのXYへ
        ]
        self._begin_sequence('投入', steps)

    def _begin_sequence(self, kind, steps):
        self._seq_kind = kind
        self._seq_steps = steps
        self._seq_index = 0
        self._seq_active = True
        self._seq_waiting_service = None
        self._seq_leg_target = None
        self._seq_leg_start_time = None
        self._seq_delay_start_time = None
        _set_status(self.sequence_status_label, f'{kind}シーケンス開始', 'muted')

    def _advance_sequence(self):
        """_spin_ros(50ms QTimer)から毎tick呼ばれる。実行中のシーケンスが無ければ
        即座に戻る(通常のGUI操作には一切影響しない)。"""
        if not self._seq_active:
            return
        if self._seq_index >= len(self._seq_steps):
            _set_status(self.sequence_status_label, f'{self._seq_kind}シーケンス完了', 'success')
            self._seq_active = False
            self._seq_steps = []
            self._seq_index = 0
            return
        step = self._seq_steps[self._seq_index]
        if step[0] == 'move':
            self._advance_move_step(step)
        elif step[0] == 'call':
            self._advance_hand_step(step)
        elif step[0] == 'wait_pick_confirm':
            self._advance_wait_pick_confirm_step(step)
        elif step[0] == 'delay':
            self._advance_delay_step(step)

    def _advance_move_step(self, step):
        # 5要素目(tip_theta)は任意。指定時のみsend_target/到達判定に加える
        # (ピック/投入シーケンス専用、_start_pick_sequence/_start_shoot_sequence参照)。
        theta, zj, r = step[1], step[2], step[3]
        tip_theta = step[4] if len(step) > 4 else None
        if self._seq_leg_target is None:
            self.node.send_target(theta, zj, r, tip_theta)
            self._seq_leg_target = (theta, zj, r, tip_theta)
            self._seq_leg_start_time = time.monotonic()
            _set_status(self.sequence_status_label,
                        f'{self._seq_kind}: 移動中 (ステップ{self._seq_index + 1}/{len(self._seq_steps)})',
                        'muted')
            return
        if not self.node.has_current_state():
            return
        pos = self.node.get_current_positions()
        reached = (
            abs(pos['root_theta_joint'] - theta) <= SEQ_MOVE_THETA_TOL
            and abs(pos['z_joint'] - zj) <= SEQ_MOVE_LINEAR_TOL
            and abs(pos['r_joint'] - r) <= SEQ_MOVE_LINEAR_TOL
            and (tip_theta is None or abs(pos['tip_theta_joint'] - tip_theta) <= SEQ_MOVE_THETA_TOL))
        if reached:
            self._seq_leg_target = None
            self._seq_index += 1
            return
        if time.monotonic() - self._seq_leg_start_time > SEQ_MOVE_TIMEOUT_SEC:
            self._abort_sequence(f'{self._seq_kind}: 移動タイムアウト(ステップ{self._seq_index + 1})')

    def _advance_delay_step(self, step):
        """指定秒数だけ待って次のステップへ進む(現状はパッド収納後のgather_
        settle_sec待ちのみ用途、_start_pick_sequence参照)。"""
        _, seconds = step
        if self._seq_delay_start_time is None:
            self._seq_delay_start_time = time.monotonic()
            _set_status(self.sequence_status_label, f'{self._seq_kind}: 待機中...', 'muted')
            return
        if time.monotonic() - self._seq_delay_start_time >= seconds:
            self._seq_delay_start_time = None
            self._seq_index += 1

    def _advance_hand_step(self, step):
        """'call'ステップ(人間の判断を要しない自動実行分)。実際の呼び出し結果は
        _on_hand_trigger_doneで受け取り、_seq_waiting_serviceと一致すれば
        _advance_sequenceが次のステップへ進める。"""
        _, service_name = step
        if self._seq_waiting_service is not None:
            return
        self._seq_waiting_service = service_name
        _set_status(self.sequence_status_label,
                    f'{self._seq_kind}: {service_name} 呼び出し中...', 'muted')
        self._on_hand_trigger(service_name)

    def _advance_wait_pick_confirm_step(self, step):
        """'wait_pick_confirm'ステップ: 人間が「回収実行」を指示する(GUIの
        「回収実行」ボタン、またはPSコンのPSボタン経由でcommand_gui_nodeの
        /pick_sequence_confirmサービスを呼ぶ、2026-09-03、ユーザー指定:「回収
        ボタンをバツ長押しからPSボタンに変更」)まで一時停止する。実際の解除は
        _on_pick_confirm_only_requestedで行う(ここでは表示のみ)。"""
        if self._seq_waiting_service is None:
            self._seq_waiting_service = '(pick_confirm)'
            _set_status(
                self.sequence_status_label,
                f'{self._seq_kind}: 回収実行の指示待ち(GUIの「回収実行」ボタンまたは'
                'PSコンのPSボタンで操作してください)', 'muted')

    def _try_confirm_pick(self):
        """wait_pick_confirmで一時停止中ならそのまま次のステップへ進める
        (=回収実行を確定する)。それ以外は何もせずFalseを返す。"""
        if self._seq_active and self._seq_waiting_service == '(pick_confirm)':
            self._seq_waiting_service = None
            self._seq_index += 1
            return True
        return False

    def _try_move_to_selected_work(self):
        """矢印キー(D-pad)で選択中のワークへの回収シーケンスを開始する
        (2026-09-03、ユーザー指定:「矢印キーでGUI上で目標ワークを選択し
        移動」)。選択カーソルは常にどこかのワークを指しているため(既定は
        先頭のワーク、マウスでのワーククリックでも同期される、_build_field_
        buttons/_on_field_point参照)基本的に常に成功するが、万一_work_grid未構築
        (GUI初期化前)なら従来の_last_pick_target(吸着失敗時の再試行用、
        2026-09-03、ユーザー指摘: 「吸着できなかったときに回収実行を再度行う
        ことがある」)にフォールバックする。
        他のシーケンス実行中(回収実行待ち・投入シーケンス中含む)でも、
        _start_pick_sequence自身が確認や中断操作なしに即座に中断して切り替える
        ため、ここでは一切ブロックしない(2026-09-03、ユーザー指摘: 「矢印での
        ワーク位置選択と移動後、回収実行をしないと移動ができない。誤って選択
        しても移動できるように回収実行を押さなくてもワーク位置移動を再度指示
        できるように」、続けて「回収実行を押さなくてもシューティング位置へ移動
        できるように。ユーザーの動きを制限したくない。状況判断はユーザーが
        行うため」により、_start_pick_sequence側の「実行中は拒否する」guardを
        撤去し常に即座に切り替わるよう変更済み。以前はここで
        `self._seq_active and self._seq_kind != '回収'`のとき投入シーケンス側の
        モーダルダイアログを避けるためにブロックしていたが、そのダイアログ自体
        が無くなったため不要になった)。"""
        target = self._selected_work_xyz()
        if target is None:
            target = self._last_pick_target
        if target is None:
            return False
        self._start_pick_sequence(*target)
        return True

    def _on_pick_confirm_requested(self):
        """GUIの「回収実行」ボタン(マウスクリック)から呼ばれる。待機中なら確定、
        非アクティブなら選択中ワークへの移動を試みる、という統合動作(マウス
        クリックには誤操作防止のボタン分離の必要が薄いため、従来通り1クリックで
        両方の役割を兼ねる)。PSコン(joy_teleop_node)は2026-09-03、ユーザー
        指定:「回収ボタンをバツ長押しからPSボタンに変更」により、この統合
        ハンドラは使わず、×ボタン=_on_pick_move_only_requested
        (pick_sequence_moveサービス)、PSボタン=_on_pick_confirm_only_requested
        (pick_sequence_confirmサービス)に分離済み(誤操作でワークに接触・吸着
        してしまうことを防ぐ安全策)。"""
        return self._try_confirm_pick() or self._try_move_to_selected_work()

    def _on_pick_confirm_only_requested(self):
        """CommandGuiNode.set_pick_confirm_handler経由、PSコン×ボタンの長押しから
        呼ばれる(2026-09-03追加)。確定のみ行い、移動は行わない
        (_on_pick_move_only_requested参照)。"""
        return self._try_confirm_pick()

    def _on_pick_move_only_requested(self):
        """CommandGuiNode.set_pick_move_handler経由、PSコン×ボタンの短押し
        (立ち上がりエッジ即時)から呼ばれる(2026-09-03追加)。選択中ワークへの
        移動のみ行い、待機中の確定は行わない(確定は長押しのみ、
        _on_pick_confirm_only_requested参照)。"""
        return self._try_move_to_selected_work()

    def _on_pick_confirm_button_clicked(self):
        if not self._on_pick_confirm_requested():
            _set_status(self.sequence_status_label, '現在「回収実行」待ちの状態ではありません', 'error')

    def _on_shoot_start_requested(self, label):
        """GUIの「L4へ移動」「R4へ移動」ボタン、またはCommandGuiNodeの
        /shoot_sequence_start_l4・_r4サービス経由(PSコンの割当ボタン、
        joy_teleop_node)から呼ばれる。SHOOT_FIXED_TARGETS[label]の固定
        シューティングエリアへ、安全高度を維持したまま向かうだけの投入シーケンスを
        実行する(2026-09-03、ユーザー指摘: 「シューティングエリアはL4もしくは
        R4で、ボタンを2つおいておいて」。以前は直前にシーケンスで向かった対象を
        覚えておいて再送信する方式だったが、実運用の対象がL4/R4の2箇所固定と
        分かったため、対象を記憶せず直接その場で指定する方式に変更した)。
        既に別のシーケンスが実行中でも、_start_shoot_sequence側が確認や中断
        操作なしに即座に中断して切り替える(2026-09-03、ユーザー指摘:「回収実行
        を押さなくてもシューティング位置へ移動できるように。ユーザーの動きを
        制限したくない」。以前はここで1件だけ予約し完了後に自動開始する
        キュー機構があったが、即座に切り替えられるようになったため不要になり
        削除した)。"""
        _, x, y, z = SHOOT_FIXED_TARGETS[label]
        self._start_shoot_sequence(x, y, z)
        return True

    def _abort_sequence(self, message):
        self._seq_active = False
        self._seq_steps = []
        self._seq_index = 0
        self._seq_waiting_service = None
        self._seq_leg_target = None
        self._seq_delay_start_time = None
        _set_status(self.sequence_status_label, message, 'error')

    def _on_abort_sequence(self):
        if not self._seq_active:
            _set_status(self.sequence_status_label, '実行中のシーケンスはありません', 'muted')
            return
        self._abort_sequence(f'{self._seq_kind}シーケンスを中断しました')

    def _on_emergency_stop_requested(self):
        """ソフト緊急停止(2026-09-08追加)。GUIの「緊急停止」ボタン・PSコン
        (joy_teleop_node)のPSボタン(/emergency_stopサービス経由)いずれからも
        呼ばれる。自動シーケンスの中断・ホーミングの中断・trajectory_follower_node
        のcubemars/robomas出力凍結をまとめて行う。解除は_on_clear_emergency_stop_
        requested(GUIの「解除」ボタンのみ)からしか行えない(誤操作で即再始動しない
        よう、緊急停止ボタン自体はトグルにしていない)。"""
        if self._seq_active:
            self._abort_sequence('緊急停止によりシーケンスを中断しました')
        # 未実行時も無条件成功で返るだけなので結果は無視してよい(homing_node.
        # _on_stop_homing参照)。homing_node未起動でも安全に無視できる。
        self.node.call_trigger_service('/stop_homing', lambda success, message: None)
        # trajectory_follower_node未起動時もcall_trigger_serviceが同期的に
        # on_done(False, ...)を呼ぶため、ここでの戻り値チェックは不要
        # (_on_estop_engage_doneがどちらの場合もestop_status_labelを更新する)。
        self.node.call_trigger_service('/engage_estop', self._on_estop_engage_done)

    def _on_estop_engage_done(self, success, message):
        _set_status(self.estop_status_label, f'緊急停止: {message}', 'error')

    def _on_clear_emergency_stop_requested(self):
        """緊急停止の解除(2026-09-08追加)。ホーミング・自動シーケンスは自動で
        再開しない(安全のため、必要ならユーザーが個別に再度開始すること)。"""
        self.node.call_trigger_service('/release_estop', self._on_estop_release_done)

    def _on_estop_release_done(self, success, message):
        _set_status(self.estop_status_label, f'解除: {message}', 'success' if success else 'error')

    def _update_status_leds(self):
        """状態表示灯(黄色/赤色LED)の表示を更新する(2026-09-08追加、1秒周期の
        _refresh_machine_statusから呼ばれる。点滅アニメーション自体はLedIndicator
        Widget側の内蔵タイマーが行うため、ここではロジック状態の設定のみ)。
        優先順位はnote/note_soki/can_mapping.txt「## 状態表示灯」の表と一致させる
        こと(複数の状態が同時に該当する場合は上位を優先表示)。"""
        # 黄色LED(注意系): シーケンス実行中 > ホーミング中 > 未ホーミング > 通常
        if self._seq_active:
            yellow = LedIndicatorWidget.STATE_BLINK_FAST
        else:
            # 文字列はhoming_node.pyのSTATE_*定数と一致させること。pausing_z/
            # pausing_rはpause_robomas_output応答待ち(2026-09-08追加、モータは
            # まだ動いていないがユーザー視点では「ホーミング中」に含めてよい)。
            homing_state = self.node.get_homing_state()
            if homing_state in ('pausing_z', 'pausing_r', 'homing_z', 'homing_r'):
                yellow = LedIndicatorWidget.STATE_BLINK_SLOW
            elif homing_state == 'done':
                yellow = LedIndicatorWidget.STATE_OFF
            else:  # None(未受信/homing_node未起動)・'idle'・'failed'
                yellow = LedIndicatorWidget.STATE_ON
        self.yellow_led.set_state(yellow)

        # 赤色LED(異常系): 緊急停止中 > ノード未起動 > リミットスイッチ安全停止中 > 異常なし
        if self.node.get_estop_active():
            red = LedIndicatorWidget.STATE_BLINK_FAST
        elif self.node.get_stale_device_ids():
            red = LedIndicatorWidget.STATE_BLINK_SLOW
        elif self.node.get_limit_stop_active():
            red = LedIndicatorWidget.STATE_ON
        else:
            red = LedIndicatorWidget.STATE_OFF
        self.red_led.set_state(red)
        self.node.publish_led_states(yellow, red)

    def _build_trajectory_panel(self, column):
        box = QGroupBox('軌道生成パラメータ (trajectory_follower_node)')
        grid = QGridLayout(box)
        grid.addWidget(QLabel('max_vel'), 0, 1)
        grid.addWidget(QLabel('max_accel'), 0, 2)
        grid.addWidget(QLabel('max_decel'), 0, 3)

        self.traj_vel_edits = {}
        self.traj_accel_edits = {}
        self.traj_decel_edits = {}
        for i, name in enumerate(TRAJ_PANEL_JOINT_NAMES):
            # 行ラベルは"_joint"を省いて表示(ボックス見出しで対象は自明なため、
            # 列幅を無駄に広げないようにする)。辞書キーは元のjoint名のまま。
            grid.addWidget(QLabel(name.removesuffix('_joint')), i + 1, 0)
            vel_edit = make_float_edit(0.0, width=70)
            accel_edit = make_float_edit(0.0, width=70)
            # 減速度(max_decel、2026-09-07新規: 停止時の応答性向上のため
            # 加速度と別値にできるようにした。trajectory_follower_node.py
            # trap_step/move_time参照)。
            decel_edit = make_float_edit(0.0, width=70)
            self.traj_vel_edits[name] = vel_edit
            self.traj_accel_edits[name] = accel_edit
            self.traj_decel_edits[name] = decel_edit
            grid.addWidget(vel_edit, i + 1, 1)
            grid.addWidget(accel_edit, i + 1, 2)
            grid.addWidget(decel_edit, i + 1, 3)

        self.traj_status_label = QLabel()
        self.traj_status_label.setWordWrap(True)
        _set_status(self.traj_status_label, '未読込', 'muted')
        grid.addWidget(self.traj_status_label, len(TRAJ_PANEL_JOINT_NAMES) + 1, 0, 1, 4)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'primary')
        load_btn.clicked.connect(self._on_load_traj_params)
        apply_btn.clicked.connect(self._on_apply_traj_params)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        grid.addLayout(btn_row, len(TRAJ_PANEL_JOINT_NAMES) + 2, 0, 1, 4)

        column.addWidget(box)

    def _build_joy_speed_panel(self, column):
        box = QGroupBox('手動操作(joy)速度 (joy_teleop_node)')
        grid = QGridLayout(box)
        self.joy_speed_edits = {}
        # tip_theta_speedは2026-09-03追加(joy_teleop_nodeのtip_theta_joint手動
        # ジョグに合わせて、この編集欄も必要になった)。
        # low_speed_multiplierはjoy_teleop_node(SHAREボタンでの低速モード)と
        # trajectory_follower_node(速度モードのmax_velocityへの乗率)の両方に
        # 同じ値を持たせる必要があるため、_on_apply_joy_speedで両ノードへ送る
        # (_on_apply_joy_speed参照)。
        fields = (
            ('theta_speed', 'root_theta', 'rad/s', 0.0),
            ('z_speed', 'z', 'm/s', 0.0),
            ('r_speed', 'r', 'm/s', 0.0),
            ('tip_theta_speed', 'tip_theta', 'rad/s', 0.0),
            # gains.json未保存の初回起動時、0.0のままだとlow_speed_multiplierの
            # ノード側バリデーション((0.0, 1.0]必須)に弾かれてしまうため、
            # 他のjoy速度と異なり有効な既定値を入れておく。
            ('low_speed_multiplier', '低速モード倍率', '×', 0.3),
        )
        for i, (name, label, unit, default) in enumerate(fields):
            grid.addWidget(QLabel(f'{label} [{unit}]'), i, 0)
            edit = make_float_edit(default, width=70)
            self.joy_speed_edits[name] = edit
            grid.addWidget(edit, i, 1)

        self.joy_speed_status_label = QLabel()
        self.joy_speed_status_label.setWordWrap(True)
        _set_status(self.joy_speed_status_label, '未読込', 'muted')
        grid.addWidget(self.joy_speed_status_label, len(fields), 0, 1, 2)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'primary')
        load_btn.clicked.connect(self._on_load_joy_speed)
        apply_btn.clicked.connect(self._on_apply_joy_speed)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        grid.addLayout(btn_row, len(fields) + 1, 0, 1, 2)

        column.addWidget(box)

    def _build_mit_gain_panel(self, column):
        # cubemars_joint_names(実機出力対象の関節)はtrajectory_follower_nodeの
        # 起動構成次第で一部の関節だけのことがある(例: root_thetaのみ)ため、
        # 軌道生成パラメータパネルと同じく実際のjoint_names順に読込・適用する。
        box = QGroupBox('MITゲイン (実機CubeMars、trajectory_follower_node)')
        grid = QGridLayout(box)
        grid.addWidget(QLabel('Kp'), 0, 1)
        grid.addWidget(QLabel('Kd'), 0, 2)
        grid.addWidget(QLabel('torque_ff'), 0, 3)

        self.mit_kp_edits = {}
        self.mit_kd_edits = {}
        self.mit_torque_edits = {}
        for i, name in enumerate(CUBEMARS_JOINT_NAMES):
            grid.addWidget(QLabel(name.removesuffix('_joint')), i + 1, 0)
            kp_edit = make_float_edit(0.0, width=60)
            kd_edit = make_float_edit(0.0, width=60)
            tff_edit = make_float_edit(0.0, width=60)
            self.mit_kp_edits[name] = kp_edit
            self.mit_kd_edits[name] = kd_edit
            self.mit_torque_edits[name] = tff_edit
            grid.addWidget(kp_edit, i + 1, 1)
            grid.addWidget(kd_edit, i + 1, 2)
            grid.addWidget(tff_edit, i + 1, 3)

        self.mit_gain_status_label = QLabel()
        self.mit_gain_status_label.setWordWrap(True)
        _set_status(self.mit_gain_status_label, '未読込', 'muted')
        grid.addWidget(self.mit_gain_status_label, len(CUBEMARS_JOINT_NAMES) + 1, 0, 1, 4)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'danger')
        load_btn.clicked.connect(self._on_load_mit_gains)
        apply_btn.clicked.connect(self._on_apply_mit_gains)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        grid.addLayout(btn_row, len(CUBEMARS_JOINT_NAMES) + 2, 0, 1, 4)

        column.addWidget(box)

    def _build_machine_origin_offset_panel(self, column):
        box = QGroupBox('機体原点オフセット (soki_sim.urdf.xacro)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, f'base_link(旋回軸)から実機の機体原点までのズレ[m]。\n'
                          f'ワーク・シューティングボックスもこのオフセットに\n'
                          f'追従して動く(可動範囲: 各軸±{MACHINE_ORIGIN_OFFSET_LIMIT:.2f}m)。', 'muted')
        layout.addWidget(desc)

        self.machine_origin_edits = {name: make_float_edit(0.0, width=70) for name in MACHINE_ORIGIN_JOINT_NAMES}
        entries = QHBoxLayout()
        for label, name in (
                ('X', 'machine_origin_x_joint'),
                ('Y', 'machine_origin_y_joint'),
                ('Z', 'machine_origin_z_joint')):
            entries.addWidget(QLabel(label))
            entries.addWidget(self.machine_origin_edits[name])
        layout.addLayout(entries)

        self.machine_origin_status_label = QLabel()
        self.machine_origin_status_label.setWordWrap(True)
        _set_status(self.machine_origin_status_label, '未送信', 'muted')
        layout.addWidget(self.machine_origin_status_label)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('現在値を反映')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'primary')
        load_btn.clicked.connect(self._on_load_machine_origin)
        apply_btn.clicked.connect(self._on_apply_machine_origin)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        layout.addLayout(btn_row)

        column.addWidget(box)

    def _on_load_machine_origin(self):
        if not self.node.has_current_state():
            QMessageBox.information(self, '未取得', 'まだmixed_joint_statesを受信していません')
            return
        pos = self.node.get_current_positions()
        for name, edit in self.machine_origin_edits.items():
            set_float(edit, round(pos.get(name, 0.0), 4))
        _set_status(self.machine_origin_status_label, '現在値を反映しました', 'info')

    def _on_apply_machine_origin(self):
        try:
            raw = {name: get_float(self.machine_origin_edits[name]) for name in MACHINE_ORIGIN_JOINT_NAMES}
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'X/Y/Zに数値を入力してください')
            return
        limit = MACHINE_ORIGIN_OFFSET_LIMIT
        clamped = {name: clamp(v, -limit, limit) for name, v in raw.items()}
        for name, v in clamped.items():
            set_float(self.machine_origin_edits[name], round(v, 4))
        self.node.send_machine_origin(*(clamped[name] for name in MACHINE_ORIGIN_JOINT_NAMES))
        x, y, z = (clamped[name] for name in MACHINE_ORIGIN_JOINT_NAMES)
        text = f'送信しました (x={x:.3f}, y={y:.3f}, z={z:.3f})'
        if any(abs(raw[name] - clamped[name]) > 1e-9 for name in MACHINE_ORIGIN_JOINT_NAMES):
            text += '\n(可動範囲外のためクランプされました)'
        _set_status(self.machine_origin_status_label, text, 'success')

    def _build_hand_offset_panel(self, column):
        box = QGroupBox('ハンド取付オフセット (soki_sim.urdf.xacro)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, f'tip_link(手先)から実機のハンド取付位置までのズレ[m]。\n'
                          f'(可動範囲: 各軸±{HAND_OFFSET_LIMIT:.2f}m)。', 'muted')
        layout.addWidget(desc)

        self.hand_offset_edits = {name: make_float_edit(0.0, width=70) for name in HAND_OFFSET_JOINT_NAMES}
        entries = QHBoxLayout()
        for label, name in (
                ('X', 'hand_offset_x_joint'),
                ('Y', 'hand_offset_y_joint'),
                ('Z', 'hand_offset_z_joint')):
            entries.addWidget(QLabel(label))
            entries.addWidget(self.hand_offset_edits[name])
        layout.addLayout(entries)

        self.hand_offset_status_label = QLabel()
        self.hand_offset_status_label.setWordWrap(True)
        _set_status(self.hand_offset_status_label, '未送信', 'muted')
        layout.addWidget(self.hand_offset_status_label)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('現在値を反映')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'primary')
        load_btn.clicked.connect(self._on_load_hand_offset)
        apply_btn.clicked.connect(self._on_apply_hand_offset)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        layout.addLayout(btn_row)

        column.addWidget(box)

    def _on_load_hand_offset(self):
        if not self.node.has_current_state():
            QMessageBox.information(self, '未取得', 'まだmixed_joint_statesを受信していません')
            return
        pos = self.node.get_current_positions()
        for name, edit in self.hand_offset_edits.items():
            set_float(edit, round(pos.get(name, 0.0), 4))
        _set_status(self.hand_offset_status_label, '現在値を反映しました', 'info')

    def _on_apply_hand_offset(self):
        try:
            raw = {name: get_float(self.hand_offset_edits[name]) for name in HAND_OFFSET_JOINT_NAMES}
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'X/Y/Zに数値を入力してください')
            return
        limit = HAND_OFFSET_LIMIT
        clamped = {name: clamp(v, -limit, limit) for name, v in raw.items()}
        for name, v in clamped.items():
            set_float(self.hand_offset_edits[name], round(v, 4))
        self.node.send_hand_offset(*(clamped[name] for name in HAND_OFFSET_JOINT_NAMES))
        x, y, z = (clamped[name] for name in HAND_OFFSET_JOINT_NAMES)
        text = f'送信しました (x={x:.3f}, y={y:.3f}, z={z:.3f})'
        if any(abs(raw[name] - clamped[name]) > 1e-9 for name in HAND_OFFSET_JOINT_NAMES):
            text += '\n(可動範囲外のためクランプされました)'
        _set_status(self.hand_offset_status_label, text, 'success')

    def _build_velocity_mode_panel(self, column):
        # joyのz/r出力を位置目標(台形プロファイル経由のMIT位置PD制御)ではなく、
        # trajectory_follower_nodeの速度モード(firmware側の速度PID、robomas_vel_kp/
        # ki/kd/max_current_a)へ直接の速度指令として送るモード(2026-09-09追加、
        # note/note_soki/hardware_mapping.txt参照)。位置モードより応答が速い反面、
        # 速度PIDのチューニング状況に依存する。チェックはjoy_teleop_node
        # (velocity_mode_enabled)とtrajectory_follower_node(robomas_velocity_mode)
        # 両方のパラメータを同時に切り替える(両者が揃っていないと、joyの速度指令が
        # 送られてもtrajectory_follower_node側は位置モードのままで無視される)。
        box = QGroupBox('joy速度指令モード')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, 'チェックを入れると、joyのz/rスティック入力(関節モード時のみ、\n'
                          'XY移動モード中のr軸は対象外)を位置目標ではなく速度指令として\n'
                          '直接送る。速度PID(robomas_vel_kp/ki/kd/max_current_a、ros2 param\n'
                          'setで調整)のチューニング状況に応答が依存する。', 'muted')
        layout.addWidget(desc)

        self.velocity_mode_check = QCheckBox('joy出力を速度指令にする')
        self.velocity_mode_check.toggled.connect(self._on_velocity_mode_toggled)
        layout.addWidget(self.velocity_mode_check)

        self.velocity_mode_status_label = QLabel()
        self.velocity_mode_status_label.setWordWrap(True)
        _set_status(self.velocity_mode_status_label, '位置指令モード', 'muted')
        layout.addWidget(self.velocity_mode_status_label)

        column.addWidget(box)

    def _on_velocity_mode_toggled(self, checked):
        ok_traj = self.node.set_node_params(
            TRAJ_NODE_NAME, {'robomas_velocity_mode': checked}, self._apply_velocity_mode_result)
        ok_joy = self.node.set_node_params(
            JOY_NODE_NAME, {'velocity_mode_enabled': checked}, self._apply_velocity_mode_result)
        if not (ok_traj and ok_joy):
            _set_status(self.velocity_mode_status_label,
                        'trajectory_follower_node/joy_teleop_nodeに接続できません(未起動?)',
                        'error')
        else:
            _set_status(self.velocity_mode_status_label, '適用中...', 'muted')

    def _apply_velocity_mode_result(self, results):
        if results is None:
            _set_status(self.velocity_mode_status_label, '適用失敗(通信エラー)', 'error')
            return
        if all(r.successful for r in results):
            checked = self.velocity_mode_check.isChecked()
            _set_status(self.velocity_mode_status_label,
                        '速度指令モード' if checked else '位置指令モード',
                        'success' if checked else 'muted')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.velocity_mode_status_label, f'適用失敗: {reasons}', 'error')

    def _build_robomas_vel_gain_panel(self, column):
        # z/rの速度モード(joy速度指令モード、上の_build_velocity_mode_panel参照)で
        # 使う速度PID(firmware側robomas.cppのROBOMAS_MODE_VELOCITY分岐)のゲイン・
        # 電流上限。以前はfirmware(config.hpp)のコンパイル時固定値だったが、
        # MITゲインと同様にCAN経由でROSから可変にした(2026-09-09追加、
        # trajectory_follower_node.pyのrobomas_vel_kp宣言部コメント参照)。
        # motor1/motor2(z/r)共通のスカラー値。
        box = QGroupBox('速度モードゲイン (joy速度指令モード用)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, 'motor1/motor2(z/r)共通の値。上の「joy速度指令モード」\n'
                          'チェックがONのときだけ実際に使われる。', 'muted')
        layout.addWidget(desc)

        grid = QGridLayout()
        self.robomas_vel_kp_edit = make_float_edit(0.8, width=70)
        self.robomas_vel_ki_edit = make_float_edit(0.0, width=70)
        self.robomas_vel_kd_edit = make_float_edit(0.0, width=70)
        self.robomas_vel_max_current_a_edit = make_float_edit(1.0, width=70)
        for i, (label, edit) in enumerate((
                ('Kp', self.robomas_vel_kp_edit),
                ('Ki', self.robomas_vel_ki_edit),
                ('Kd', self.robomas_vel_kd_edit),
                ('電流上限 [A]', self.robomas_vel_max_current_a_edit))):
            grid.addWidget(QLabel(label), i, 0)
            grid.addWidget(edit, i, 1)
        layout.addLayout(grid)

        self.robomas_vel_gain_status_label = QLabel()
        self.robomas_vel_gain_status_label.setWordWrap(True)
        _set_status(self.robomas_vel_gain_status_label, '未読込', 'muted')
        layout.addWidget(self.robomas_vel_gain_status_label)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'danger')
        load_btn.clicked.connect(self._on_load_robomas_vel_gains)
        apply_btn.clicked.connect(self._on_apply_robomas_vel_gains)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        layout.addLayout(btn_row)

        column.addWidget(box)

    def _on_load_robomas_vel_gains(self):
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME,
            ['robomas_vel_kp', 'robomas_vel_ki', 'robomas_vel_kd', 'robomas_vel_max_current_a'],
            self._apply_loaded_robomas_vel_gains,
            lambda reason: _set_status(self.robomas_vel_gain_status_label, f'読込失敗: {reason}', 'error'))
        _set_status(self.robomas_vel_gain_status_label,
                    '読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')
        return ok

    def _apply_loaded_robomas_vel_gains(self, values):
        if 'robomas_vel_kp' in values:
            set_float(self.robomas_vel_kp_edit, round(values['robomas_vel_kp'], 6))
        if 'robomas_vel_ki' in values:
            set_float(self.robomas_vel_ki_edit, round(values['robomas_vel_ki'], 6))
        if 'robomas_vel_kd' in values:
            set_float(self.robomas_vel_kd_edit, round(values['robomas_vel_kd'], 6))
        if 'robomas_vel_max_current_a' in values:
            set_float(self.robomas_vel_max_current_a_edit,
                      round(values['robomas_vel_max_current_a'], 6))
        # 「読込」はノードの現在値をGUIに表示するだけに留め、gains.jsonへは
        # 「適用」時のみ永続化する(他のゲインパネルと同じ方針)。
        _set_status(self.robomas_vel_gain_status_label, '読込完了', 'info')

    def _collect_robomas_vel_values(self):
        return {
            'robomas_vel_kp': get_float(self.robomas_vel_kp_edit),
            'robomas_vel_ki': get_float(self.robomas_vel_ki_edit),
            'robomas_vel_kd': get_float(self.robomas_vel_kd_edit),
            'robomas_vel_max_current_a': get_float(self.robomas_vel_max_current_a_edit),
        }

    def _on_apply_robomas_vel_gains(self):
        try:
            values = self._collect_robomas_vel_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'Kp/Ki/Kd/電流上限に数値を入力してください')
            return
        reply = QMessageBox.question(
            self, '速度モードゲイン適用の確認',
            'motor1/motor2(z/r)の速度モードゲイン・電流上限を実機へ即座に反映します。\n'
            '「joy速度指令モード」がONの間は実際の動きにすぐ影響します。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self._persist_gains('robomas_vel_gain', values)
        ok = self.node.set_node_params(TRAJ_NODE_NAME, values, self._apply_robomas_vel_gain_set_result)
        _set_status(self.robomas_vel_gain_status_label,
                    '適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')

    def _apply_robomas_vel_gain_set_result(self, results):
        if results is None:
            _set_status(self.robomas_vel_gain_status_label, '適用に失敗しました(応答なし)', 'error')
            return
        if all(r.successful for r in results):
            _set_status(self.robomas_vel_gain_status_label, '適用しました', 'success')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.robomas_vel_gain_status_label, f'適用失敗: {reasons}', 'error')

    def _build_axis_enable_panel(self, column):
        # 組立中(まだ配線・組付けが終わっていない軸がある)や実機の不具合発生時に、
        # その軸だけソフトウェア側で無視できるようにするパネル(2026-09-05追加)。
        # チェックを外すとtrajectory_follower_nodeのdisabled_jointsパラメータに
        # 追加され、以後その軸への目標追従を止めて現在位置で凍結する(他の軸は
        # 通常通り動く)。モータへのMIT指令自体は現在位置保持として送り続けるため、
        # 「軸を無視」であって「モータへの通電を止める」わけではない点に注意
        # (z_joint/r_jointはロボマス差動のため、そもそも片方だけの通電停止はできない)。
        box = QGroupBox('軸の有効/無効 (組立中・不具合時に軸を無視)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, 'チェックを外すと、その軸への目標追従を止めて現在位置で\n'
                          '保持する(他の軸は通常通り動作する。モータへの指令自体は\n'
                          '現在位置保持として送り続ける)。', 'muted')
        layout.addWidget(desc)

        grid = QGridLayout()
        self.axis_enable_checks = {}
        for i, name in enumerate(TRAJ_PANEL_JOINT_NAMES):
            cb = QCheckBox(name.removesuffix('_joint'))
            cb.setChecked(True)
            cb.toggled.connect(self._on_axis_enable_toggled)
            self.axis_enable_checks[name] = cb
            grid.addWidget(cb, i // 2, i % 2)
        layout.addLayout(grid)

        self.axis_enable_status_label = QLabel()
        self.axis_enable_status_label.setWordWrap(True)
        _set_status(self.axis_enable_status_label, '全軸有効', 'muted')
        layout.addWidget(self.axis_enable_status_label)

        column.addWidget(box)

    def _on_axis_enable_toggled(self, _checked=None):
        disabled = [name for name, cb in self.axis_enable_checks.items() if not cb.isChecked()]
        ok = self.node.set_node_params(TRAJ_NODE_NAME, {'disabled_joints': disabled},
                                        self._apply_axis_enable_result)
        _set_status(self.axis_enable_status_label,
                    '適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')

    def _apply_axis_enable_result(self, results):
        if results is None:
            _set_status(self.axis_enable_status_label, '適用に失敗しました(応答なし)', 'error')
            return
        if all(r.successful for r in results):
            disabled = [name.removesuffix('_joint') for name, cb in self.axis_enable_checks.items()
                        if not cb.isChecked()]
            if disabled:
                _set_status(self.axis_enable_status_label, f'無効化中: {", ".join(disabled)}', 'info')
            else:
                _set_status(self.axis_enable_status_label, '全軸有効', 'success')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.axis_enable_status_label, f'適用失敗: {reasons}', 'error')

    def _build_origin_panel(self, column):
        # 2026-09-08方針変更: tip_theta_jointはROBOMAS(M2006)側へ移行し、Set Origin
        # 機構自体を持たない(原点センサも無く、電源投入前の手動ゼロ合わせ+起動時
        # リセットされる内蔵エンコーダの値をそのまま原点として使う)ため、
        # ここはroot_theta(CubeMars AK40-10)のみを対象とする。
        box = QGroupBox('root_theta 原点設定 (CubeMars本体、trajectory_follower_node)')
        layout = QVBoxLayout(box)

        warn = QLabel()
        warn.setWordWrap(True)
        _set_status(warn, '呼び出し前に、対象の関節を原点センサの位置\n'
                          '(真の機械原点)へ物理的に合わせておくこと。', 'error')
        layout.addWidget(warn)

        self.origin_status_label = QLabel()
        self.origin_status_label.setWordWrap(True)
        _set_status(self.origin_status_label, '未実行', 'muted')
        layout.addWidget(self.origin_status_label)

        root_theta_origin_btn = QPushButton('/set_root_theta_origin 呼び出し')
        root_theta_origin_btn.setProperty('variant', 'danger')
        root_theta_origin_btn.clicked.connect(self._on_set_root_theta_origin)
        layout.addWidget(root_theta_origin_btn)

        column.addWidget(box)

    def _on_set_root_theta_origin(self):
        reply = QMessageBox.question(
            self, '根本θ原点設定の確認',
            'root_theta_jointをCubeMars本体(AK40-10)のフラッシュへ\n'
            '永久原点として書き込みます。\n\n'
            '関節は今、原点センサの位置(真の機械原点)にありますか？\n'
            '間違った位置で実行すると、以後のすべての角度がずれます。',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        _set_status(self.origin_status_label, '呼び出し中...', 'muted')
        ok = self.node.call_trigger_service(
            '/set_root_theta_origin', self._on_set_root_theta_origin_done)
        if not ok:
            _set_status(self.origin_status_label, 'サービス未起動です', 'error')

    def _on_set_root_theta_origin_done(self, success, message):
        _set_status(self.origin_status_label, message, 'success' if success else 'error')

    # ---------- real_joint_bridge.yaml配線設定(センサID・CubeMars/RoboMasのID・
    # 回転方向)----
    # Kp/Kd等と違い、この節のパラメータはreal_joint_bridge_node/homing_nodeが
    # 起動時に一度だけ読み込む「起動時設定」で、SetParametersでは実行中ノードに
    # 反映されない(モジュール先頭のROBOMAS_WIRING_FIELDS等のコメント参照)。
    # そのためros2パラメータサービスではなく、設定の保存先そのもの
    # (soki_sim/config/real_joint_bridge.yaml)を直接読み書きする。
    def _build_robomas_wiring_panel(self, column):
        self._build_yaml_wiring_panel(
            column, attr_prefix='robomas_wiring',
            title='RoboMas ID・回転方向 (real_joint_bridge.yaml)',
            note='motor1/motor2は、z_joint(昇降)・r_joint(伸縮)を差動駆動する\n'
                 '2基のRoboMas(M2006+C610)モータ(z=mix_k*(m1+m2)、r=mix_k*(m1-m2))。\n'
                 '内蔵ロータエンコーダのCAN帰還を位置の真値として使う。\n'
                 'tip_theta(M3)は同じ機種を単独直接駆動(ミックス無し)に使う\n'
                 '(2026-09-08新規、旧CubeMars AK40-10からの移行。原点センサが\n'
                 '無いため、電源投入前に機構原点(0deg)へ手で合わせておくこと)。',
            description='real_joint_bridge_node/homing_node共通。次回ノード起動から\n'
                        '反映されます(実行中には反映されません)。',
            field_specs=ROBOMAS_WIRING_FIELDS)

    def _build_cubemars_wiring_panel(self, column):
        self._build_yaml_wiring_panel(
            column, attr_prefix='cubemars_wiring',
            title='CubeMars ID・回転方向 (real_joint_bridge.yaml)',
            description='real_joint_bridge_node起動時のみ反映。実行中には反映されません。',
            field_specs=CUBEMARS_WIRING_FIELDS)

    def _build_homing_wiring_panel(self, column):
        self._build_yaml_wiring_panel(
            column, attr_prefix='homing_wiring',
            title='原点センサ・ホーミング配線設定 (real_joint_bridge.yaml)',
            note='CAN_HOSTはz/r原点センサ(SW1/SW2のリミットスイッチ)専用。\n'
                 'motor1/motor2の位置取得にはROBOMAS内蔵エンコーダを使うため、\n'
                 'CAN_HOSTのENC1/ENC2は使わない。',
            description='homing_node起動時のみ反映。実行中には反映されません。',
            field_specs=HOMING_WIRING_FIELDS)

    def _build_can_host_raw_monitor_panel(self, column):
        """上の「原点センサ・ホーミング配線設定」で設定したノード/スロットが実機と
        合っているかを確認するための生値モニタ(2026-09-08追加、ユーザー要望:
        「現在割り当てられている原点センサの対応付けがあっているか確認できるように
        GUIで表示」)。CAN_HOST(device_id=101)のserial_rx_101_unwrappedを直接
        (ros2canのtopic_passthrough経由ではなく)購読してSLOT_COUNT分すべての
        生値を一覧表示し、上のz/r原点センサ欄から計算したスロットに
        →z原点/→r原点の印を付ける。原点センサを実機で手動操作しながらどの
        スロットの値が変化するかを見比べることで、配線設定欄の値が正しいか
        (このパネルの目的はnote/note_soki/hardware_mapping.txt「未確認事項」に
        あった原点センサ配線の実機検証を、ノード再起動・ログ確認無しでGUI上から
        直接行えるようにすること)確認できる。"""
        box = QGroupBox('原点センサ 生値モニタ (実機確認用)')
        layout = QVBoxLayout(box)

        note = QLabel()
        note.setWordWrap(True)
        _set_status(
            note,
            '上の「原点センサ・ホーミング配線設定」のノード/スロット欄に対応する\n'
            'スロットに →z原点 / →r原点 と表示する。原点センサを手で押しながら\n'
            'どのslotの値がSW検出値に変わるか見比べ、印の付いた欄と一致していれば\n'
            '設定は正しい。違うslotが反応する場合は上の欄をそのslot番号\n'
            '(ノード=slot÷ノードあたりスロット数、スロット=slot mod ノードあたり\n'
            'スロット数)に書き換えて保存すること。',
            'muted')
        layout.addWidget(note)

        self.can_host_raw_status_label = QLabel()
        _set_status(self.can_host_raw_status_label, '未受信', 'muted')
        layout.addWidget(self.can_host_raw_status_label)

        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(2)
        self.can_host_raw_slot_labels = {}
        cols = 4
        for slot in range(CAN_HOST_RAW_SLOT_COUNT):
            r, c = divmod(slot, cols)
            label = QLabel()
            _set_status(label, f'slot{slot}: -', 'muted')
            self.can_host_raw_slot_labels[slot] = label
            grid.addWidget(label, r, c)
        layout.addLayout(grid)

        column.addWidget(box)

        # 実機のSW押下に追従して見えるよう、他の状態パネル(_machine_status_timer、
        # 1000ms間隔)より短い周期で更新する専用タイマー。
        self._can_host_raw_timer = QTimer(self)
        self._can_host_raw_timer.timeout.connect(self._refresh_can_host_raw_monitor)
        self._can_host_raw_timer.start(150)

    def _refresh_can_host_raw_monitor(self):
        edits = self._homing_wiring_edits
        try:
            device_id = get_int(edits['can_host_device_id'])
            slots_per_node = get_int(edits['can_host_slots_per_node'])
            triggered_value = get_int(edits['switch_triggered_value'])
            z_slot = (get_int(edits['z_limit_switch_node_index']) * slots_per_node
                      + get_int(edits['z_limit_switch_local_index']))
            r_slot = (get_int(edits['r_limit_switch_node_index']) * slots_per_node
                      + get_int(edits['r_limit_switch_local_index']))
        except ValueError:
            _set_status(self.can_host_raw_status_label, '上の配線設定欄の入力エラー', 'error')
            return

        data = self.node.get_device_raw_slots(device_id)
        if data is None:
            _set_status(self.can_host_raw_status_label,
                        f'device_id={device_id}のserial_rx未受信(CAN_HOST起動・'
                        'topic_passthrough ON確認)', 'error')
            for slot, label in self.can_host_raw_slot_labels.items():
                _set_status(label, f'slot{slot}: -', 'muted')
            return
        _set_status(self.can_host_raw_status_label,
                    f'device_id={device_id} 受信中 (ノードあたりスロット数={slots_per_node})',
                    'success')

        for slot, label in self.can_host_raw_slot_labels.items():
            if slot >= len(data):
                _set_status(label, f'slot{slot}: -', 'muted')
                continue
            value = data[slot]
            node_idx, local_idx = ((slot // slots_per_node, slot % slots_per_node)
                                    if slots_per_node > 0 else (0, slot))
            tag = ''
            if slot == z_slot:
                tag += ' →z原点'
            if slot == r_slot:
                tag += ' →r原点'
            text = f'slot{slot}(n{node_idx}/{local_idx}): {value}{tag}'
            if tag:
                role = 'success' if value == triggered_value else 'error'
            else:
                role = 'info' if value == triggered_value else 'muted'
            _set_status(label, text, role)

    def _build_limit_switch_wiring_panel(self, column):
        self._build_yaml_wiring_panel(
            column, attr_prefix='limit_switch_wiring',
            title='z/r安全停止センサ配線設定 (real_joint_bridge.yaml)',
            note='原点較正(ホーミング)用の下限センサとは別に、z/r軸それぞれの\n'
                 '上限・下限リミットスイッチをtrajectory_follower_nodeが直接監視し、\n'
                 'トリガーされた方向への移動だけをロックする(過走防止)。\n'
                 'IDを0のままにするとそのスイッチは無効(未配線)扱い。',
            description='trajectory_follower_node起動時のみ反映。実行中には反映されません。',
            field_specs=LIMIT_SWITCH_WIRING_FIELDS)

    def _build_hand_wiring_panel(self, column):
        self._build_yaml_wiring_panel(
            column, attr_prefix='hand_wiring',
            title='ハンド配線設定 (hand.yaml)',
            note='吸着パッド展開・ワークピッチ変更の2サーボ(SERVOn、角度[deg])と、\n'
                 'ダイヤフラムポンプ(MDn、PWM+DIR)の配線・角度・デューティを設定する。\n'
                 'IDを0のままにするとその出力は無効(未配線)扱い。',
            description='hand_node起動時のみ反映。実行中には反映されません。',
            field_specs=HAND_WIRING_FIELDS,
            yaml_filename='hand.yaml',
            preview_specs=HAND_SERVO_PREVIEW_SPECS)

    def _build_yaml_wiring_panel(self, column, attr_prefix, title, description, field_specs, note=None,
                                  yaml_filename='real_joint_bridge.yaml', preview_specs=None):
        box = QGroupBox(title)
        layout = QVBoxLayout(box)

        if note:
            note_label = QLabel()
            note_label.setWordWrap(True)
            _set_status(note_label, note, 'muted')
            layout.addWidget(note_label)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, description, 'error')
        layout.addWidget(desc)

        # field_specsは[(行見出し またはNone, [(key,ラベル,型), ...]), ...]。
        # 1タプル=1行で、ID・ノード・スロット等の関連する項目がまとめて
        # 横並びになるようにする(2026-08-31、それまでは項目数で機械的に
        # 2列へ折り返していたため、関連項目が別行に分かれることがあった)。
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        edits = {}
        for r, (row_title, fields) in enumerate(field_specs):
            c = 0
            if row_title:
                grid.addWidget(QLabel(row_title), r, c)
                c += 1
            for key, label, kind in fields:
                grid.addWidget(QLabel(label), r, c)
                if kind == 'bool':
                    widget = QCheckBox()
                elif kind == 'int':
                    # ID/ノード/スロットは小さい数値(数桁)なので70pxは無駄に広い。
                    widget = make_int_edit(0, width=44)
                else:
                    widget = make_float_edit(0.0, width=56)
                edits[key] = widget
                grid.addWidget(widget, r, c + 1)
                c += 2
        layout.addLayout(grid)
        setattr(self, f'_{attr_prefix}_edits', edits)

        if preview_specs:
            self._build_wiring_preview(layout, edits, preview_specs)

        status_label = QLabel()
        status_label.setWordWrap(True)
        _set_status(status_label, '未読込', 'muted')
        setattr(self, f'_{attr_prefix}_status_label', status_label)
        layout.addWidget(status_label)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        save_btn = QPushButton('yamlへ保存')
        save_btn.setProperty('variant', 'danger')
        load_btn.clicked.connect(lambda: self._on_load_yaml_wiring(attr_prefix, field_specs, yaml_filename))
        save_btn.clicked.connect(lambda: self._on_save_yaml_wiring(attr_prefix, field_specs, title, yaml_filename))
        btn_row.addWidget(load_btn)
        btn_row.addWidget(save_btn)
        layout.addLayout(btn_row)

        column.addWidget(box)

        # 一括読込パネル(_build_wiring_bulk_load_panel)向けの登録と、GUI起動時の
        # 自動読込(2026-09-07新規、ユーザー指摘: 「配線設定をオートで読み込んで
        # ほしい」)。従来は「未読込」のまま放置され、各パネルで毎回「読込」を
        # 押す必要があった。
        if not hasattr(self, '_yaml_wiring_panels'):
            self._yaml_wiring_panels = []
        self._yaml_wiring_panels.append((attr_prefix, field_specs, yaml_filename, title))
        self._on_load_yaml_wiring(attr_prefix, field_specs, yaml_filename)

    def _build_wiring_preview(self, layout, edits, preview_specs):
        """preview_specs: (角度キー, オフセットキー, オーバーライド有効キー,
        オーバーライド角度キー, ラベル)のリスト。オーバーライドが有効なら
        その角度をそのまま、無効なら論理値[deg](sim表示用、無制限)+オフセットを
        「実機に送信する角度」として表示する読み取り専用プレビュー(2026-09-03、
        クランプは行わない。hand_node.py _resolve_can_deg参照)。editsの該当欄が
        変わるたびに自動更新する(setText()もtextChangedを発火するため、yaml
        読込時の一括反映でも追従する)。"""
        grid = QGridLayout()
        preview_labels = {}
        for i, (deg_key, offset_key, override_key, override_deg_key, label) in enumerate(preview_specs):
            grid.addWidget(QLabel(label), i, 0)
            value_label = QLabel()
            preview_labels[(deg_key, offset_key, override_key, override_deg_key)] = value_label
            grid.addWidget(value_label, i, 1)
        layout.addLayout(grid)

        def _refresh(*_args):
            for (deg_key, offset_key, override_key, override_deg_key), value_label in preview_labels.items():
                try:
                    if override_key in edits and edits[override_key].isChecked():
                        can_deg = get_float(edits[override_deg_key])
                        _set_status(value_label, f'{can_deg:.1f}deg (手動固定)', 'info')
                        continue
                    if deg_key not in edits:
                        continue
                    deg = get_int(edits[deg_key])
                    offset = get_float(edits[offset_key]) if offset_key in edits else 0.0
                except ValueError:
                    _set_status(value_label, '(入力エラー)', 'error')
                    continue
                _set_status(value_label, f'{deg + offset:.1f}deg', 'info')

        for deg_key, offset_key, override_key, override_deg_key, _label in preview_specs:
            for key in (deg_key, offset_key, override_deg_key):
                if key in edits:
                    edits[key].textChanged.connect(_refresh)
            if override_key in edits:
                edits[override_key].stateChanged.connect(_refresh)
        _refresh()

    def _on_load_yaml_wiring(self, attr_prefix, field_specs, yaml_filename='real_joint_bridge.yaml'):
        """成功したかどうかをbool で返す(_on_bulk_load_yaml_wiringの集計用、
        _on_load_all_gainsの各ゲインローダーと同じ設計)。"""
        edits = getattr(self, f'_{attr_prefix}_edits')
        status_label = getattr(self, f'_{attr_prefix}_status_label')
        path = _resolve_config_yaml_path(yaml_filename)
        if path is None:
            _set_status(status_label, f'{yaml_filename}が見つかりません', 'error')
            return False
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as exc:
            _set_status(status_label, f'読込失敗: {exc}', 'error')
            return False
        values = _flatten_yaml_node_params(data)
        missing = []
        for key, _label, kind in _iter_wiring_fields(field_specs):
            if key not in values:
                missing.append(key)
                continue
            widget = edits[key]
            if kind == 'bool':
                widget.setChecked(bool(values[key]))
            elif kind == 'int':
                set_int(widget, int(values[key]))
            else:
                set_float(widget, float(values[key]))
        status = f'読込完了 ({path})'
        if missing:
            status += f'\n(yamlに無い項目: {", ".join(missing)})'
        _set_status(status_label, status, 'info')
        return True

    def _on_save_yaml_wiring(self, attr_prefix, field_specs, title, yaml_filename='real_joint_bridge.yaml'):
        edits = getattr(self, f'_{attr_prefix}_edits')
        status_label = getattr(self, f'_{attr_prefix}_status_label')
        path = _resolve_config_yaml_path(yaml_filename)
        if path is None:
            _set_status(status_label, f'{yaml_filename}が見つかりません', 'error')
            return
        try:
            updates = {}
            for key, _label, kind in _iter_wiring_fields(field_specs):
                widget = edits[key]
                if kind == 'bool':
                    updates[key] = _format_yaml_bool(widget.isChecked())
                elif kind == 'int':
                    updates[key] = _format_yaml_int(get_int(widget))
                else:
                    updates[key] = _format_yaml_float(get_float(widget))
        except ValueError:
            QMessageBox.critical(self, '入力エラー', '数値項目を確認してください')
            return

        reply = QMessageBox.question(
            self, f'{title}の保存確認',
            f'{path}\n\nを直接書き換えます。対象ノードの次回起動から反映されます\n'
            '(実行中のノードには影響しません)。git管理下のファイルです。\n'
            '保存後はgit diffで変更内容を確認してください。\n'
            'よろしいですか?',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        try:
            with open(path, 'r', encoding='utf-8') as f:
                text = f.read()
            for key, value_str in updates.items():
                text = _replace_yaml_scalar(text, key, value_str)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
        except OSError as exc:
            _set_status(status_label, f'保存失敗: {exc}', 'error')
            return
        _set_status(status_label, f'保存しました ({path})\ngit diffで変更内容を確認してください', 'success')

    def _build_field_buttons(self, layout):
        # 以前はワーク/シューティングボックスを別々のボックス(別々の座標系)で
        # 描画していたため、両者の実際の左右・奥行き関係(シューティングボックスは
        # ワークより機体側(Y-)にあり、X方向はワークの可動列よりさらに外側にある)が
        # GUI上で見た目に対応していなかった。1つのグリッドに統合し、実座標
        # (WORK_POINTS/SHOOT_POINTSのx,y)でボタン位置を決めることで、実際の
        # フィールド配置(奥からワーク4列・機体・シューティングボックス4列の順)と
        # 対応する見た目にする。
        # 「自動シーケンスで実行」ON時は、workボタンで回収シーケンス・shootボタンで
        # 投入シーケンスを開始する(_on_field_point参照)。OFFなら従来通り即時移動
        # のみ行う(2026-09-03新規、既定は当初OFFだったが、同日ユーザー指定で
        # 既定ONに変更)。
        self.seq_auto_checkbox = QCheckBox(
            '自動シーケンスで実行(ワーク=回収・シューティングボックス=投入。'
            '設定は下の「ピック/投入 自動シーケンス」パネル)')
        self.seq_auto_checkbox.setChecked(True)
        seq_row = QHBoxLayout()
        seq_row.addWidget(self.seq_auto_checkbox)
        seq_row.addStretch(1)
        layout.addLayout(seq_row)

        box = QGroupBox('ワーク・シューティングボックス (クリックで移動、実フィールド配置)')
        grid = QGridLayout(box)
        work_points = [(*p, 'pick') for row in WORK_POINTS for p in row]
        shoot_points = [(*p, 'shoot') for p in SHOOT_POINTS['L'] + SHOOT_POINTS['R']]
        n_work_rows = len({round(p[2], 6) for p in work_points})
        # 矢印キー(D-pad)でのワーク選択カーソル用に、ワークボタンのウィジェット
        # 参照を(round(x,6), round(y,6))キーで覚えておく(2026-09-03追加、
        # ユーザー指定:「矢印キーでGUI上で目標ワークを選択し移動バツで移動、
        # 再度バツで回収実行」)。on_buttonコールバック経由で_build_button_grid
        # から受け取る。
        self._work_buttons = {}

        def _register_button(label, x, y, z, kind, btn):
            if kind == 'pick':
                self._work_buttons[(round(x, 6), round(y, 6))] = btn

        self._build_button_grid(grid, work_points + shoot_points, header_row=1,
                                 gap_after_rank=n_work_rows - 1, gap_label='(機体)',
                                 on_button=_register_button)

        # ワーク選択カーソルの座標系(見た目通りのrank/col、シューティングボックス
        # 列を挟まないワーク4行6列のみのローカルな添字)。_build_button_grid内の
        # xs/ys計算と同じ基準(X昇順=左->右、Y降順=奥->手前)で独自に計算する。
        work_xs = sorted({round(p[1], 6) for row in WORK_POINTS for p in row})
        work_ys = sorted({round(p[2], 6) for row in WORK_POINTS for p in row}, reverse=True)
        self._work_grid = {}         # (rank, col) -> (x, y, z)
        self._work_grid_rc_by_xy = {}  # (round(x,6), round(y,6)) -> (rank, col)
        for wrow in WORK_POINTS:
            for _label, x, y, z in wrow:
                rc = (work_ys.index(round(y, 6)), work_xs.index(round(x, 6)))
                self._work_grid[rc] = (x, y, z)
                self._work_grid_rc_by_xy[(round(x, 6), round(y, 6))] = rc
        self._work_grid_rows = len(work_ys)
        self._work_grid_cols = len(work_xs)
        self._selected_work_rc = (0, 0)
        self._highlight_selected_work(True)

        # QVBoxLayoutへ直接addWidgetすると幅いっぱいに引き伸ばされてしまう
        # (グリッド内容は中身ぴったりのまま、右側だけ間延びした余白ができる)ため、
        # 横方向はQHBoxLayout+末尾stretchで中身ぴったりの幅に留める。
        # ピック/投入自動シーケンスパネルは、このワーク・シューティングボックス
        # パネルのすぐ隣に置く(2026-09-03、ユーザー指定。以前は統合操作タブの
        # 左列下部にあり、work/shootボタンから視線が離れていた)。
        row = QHBoxLayout()
        row.addWidget(box)
        self._build_sequence_settings_panel(row)
        row.addStretch(1)
        layout.addLayout(row)

    def _build_button_grid(self, grid, points, header_row=0, gap_after_rank=None, gap_label='',
                            on_button=None):
        """points: (label, x, y, z, kind)のリスト('pick'=ワーク/'shoot'=シューティング
        ボックス、_on_field_point参照)。実座標を見た目通りに配置する
        (X昇順=左->右の列、Y降順=奥(ワーク方向)が上->手前が下の行)。
        gap_after_rankを指定すると、Y順位でその順位を超えた行を1行分下にずらし、
        間にgap_labelを挟む(ワーク行とシューティングボックス行の間に機体分の
        空白を作るため)。on_button(label, x, y, z, kind, btn)を指定すると、
        作成した各QPushButtonを呼び出し元へ渡す(2026-09-03追加、ワーク選択
        カーソルのハイライト用にウィジェット参照を回収するため、
        _build_field_buttons参照)。"""
        grid.setHorizontalSpacing(4)
        xs = sorted({round(p[1], 6) for p in points})
        ys = sorted({round(p[2], 6) for p in points}, reverse=True)
        # ヘッダーラベルのcolSpanは実際の列数ぴったりにする。以前は"とりあえず
        # 十分大きい値"として99を指定していたが、QGridLayoutはcolSpanの終端列まで
        # 実在する列として扱うため、ボタンが無い列が90個以上生まれ、
        # setHorizontalSpacing(4)による列間隙間(4px×約98列分)がそのまま
        # ボックスの余分な横幅になっていた(ワーク/シューティングボックスが
        # 中身に対して不自然に広く見えていた原因)。
        ncols = len(xs)
        grid.addWidget(QLabel('← X- ・ X+ →'), 0, 0, 1, ncols)
        # ラベル文字列("4-6"等)がボタン内に収まるよう、実際の文字幅+variant="grid"の
        # 詰めたpadding(QSS参照。4px*2+border2px)分の余白から幅を決める。
        # 固定48pxだと桁数が増えたラベルが見切れていた。
        metrics = QFontMetrics(QPushButton().font())
        btn_width = max(metrics.horizontalAdvance(label) for label, _, _, _, _ in points) + 14
        for label, x, y, z, kind in points:
            col = xs.index(round(x, 6))
            rank = ys.index(round(y, 6))
            if gap_after_rank is not None and rank > gap_after_rank:
                rank += 1
            btn = QPushButton(label)
            # 3個ずつ回収する運用では、行内の列2・列5を狙うと両端1列ずつ含めて
            # ちょうど3個(列1-3・列4-6)を吸着パッドでカバーできる(2026-09-03、
            # ユーザー指定: 「3つずつとる想定でX-2,X-5のワークボタンは色を
            # 変えておいて」)。該当するワークボタンだけ目立つ色にして、押すべき
            # ボタンが一目でわかるようにする。
            col_label = label.rsplit('-', 1)[-1]
            variant = 'grid-highlight' if (kind == 'pick' and col_label in ('2', '5')) else 'grid'
            btn.setProperty('variant', variant)
            btn.setFixedWidth(btn_width)
            btn.clicked.connect(
                lambda _checked=False, x=x, y=y, z=z, kind=kind: self._on_field_point(x, y, z, kind))
            grid.addWidget(btn, rank + header_row, col)
            if on_button is not None:
                on_button(label, x, y, z, kind, btn)

        has_gap = gap_after_rank is not None
        if has_gap and gap_label:
            gap_widget = QLabel(gap_label)
            gap_widget.setAlignment(Qt.AlignCenter)
            _set_status(gap_widget, gap_label, 'muted')
            grid.addWidget(gap_widget, header_row + gap_after_rank + 1, 0, 1, ncols)

        total_rows = len(ys) + (1 if has_gap else 0)
        grid.addWidget(QLabel('↑ Y+ (ワーク側) ／ Y- (機体側) ↓'), header_row + total_rows, 0, 1, ncols)

    def _on_field_point(self, x, y, z, kind):
        # ワークボタンをマウスでクリックした場合も、矢印キー(D-pad)の選択
        # カーソルを同じワークへ同期させる(2026-09-03追加。これによりPSコンの
        # ×ボタンでの「移動」「回収実行」がマウス操作と矛盾しない)。
        if kind == 'pick':
            rc = self._work_grid_rc_by_xy.get((round(x, 6), round(y, 6)))
            if rc is not None and rc != self._selected_work_rc:
                self._highlight_selected_work(False)
                self._selected_work_rc = rc
                self._highlight_selected_work(True)
        if self.seq_auto_checkbox.isChecked():
            if kind == 'pick':
                self._start_pick_sequence(x, y, z)
            else:
                self._start_shoot_sequence(x, y, z)
        else:
            self._send_xyz(x, y, z)

    def _highlight_selected_work(self, selected):
        """現在のワーク選択カーソル(self._selected_work_rc)が指すボタンの
        'selected'プロパティを更新し、QSSのQPushButton[selected="true"]
        (style.qss)でハイライト枠を付け外しする(2026-09-03追加)。プロパティを
        setProperty()するだけではQtがスタイルを再適用しないため、
        unpolish/polishで強制的に反映させる。"""
        target = self._work_grid.get(self._selected_work_rc)
        if target is None:
            return
        x, y, _z = target
        btn = self._work_buttons.get((round(x, 6), round(y, 6)))
        if btn is None:
            return
        btn.setProperty('selected', selected)
        btn.style().unpolish(btn)
        btn.style().polish(btn)

    def _move_work_selection(self, d_rank, d_col):
        """矢印キー(D-pad)でワーク選択カーソルを1マス動かす(2026-09-03追加、
        ユーザー指定:「矢印キーでGUI上で目標ワークを選択し移動」)。範囲外へは
        動かず(clampするだけ)、末尾の列/行で押し続けても他の行/列へ回り込んだり
        しない。"""
        row, col = self._selected_work_rc
        new_rc = (
            int(clamp(row + d_rank, 0, self._work_grid_rows - 1)),
            int(clamp(col + d_col, 0, self._work_grid_cols - 1)),
        )
        if new_rc == self._selected_work_rc:
            return
        self._highlight_selected_work(False)
        self._selected_work_rc = new_rc
        self._highlight_selected_work(True)

    def _on_work_select_requested(self, direction):
        """CommandGuiNode.set_work_select_handler経由(PSコンの十字キー、
        joy_teleop_node)から呼ばれる(2026-09-03追加)。"""
        d_rank, d_col = {'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1)}[direction]
        self._move_work_selection(d_rank, d_col)

    def _selected_work_xyz(self):
        """現在のワーク選択カーソルが指すワークの(x, y, z)を返す(2026-09-03追加、
        _on_pick_confirm_requested参照)。"""
        return self._work_grid.get(self._selected_work_rc)

    # ---------- realtime state / trajectory params ----------
    def _spin_ros(self):
        # rclpy.spin_once()はmixed_joint_states購読・パラメータサービスの
        # 応答処理に必要(このメソッドの呼び出し=QTimer=Qtのイベントループと
        # 同じメインスレッド上で完結する)。
        rclpy.spin_once(self.node, timeout_sec=0)
        self._refresh_current_state()
        self._refresh_sequence_pump_status()
        self._advance_sequence()
        self._refresh_status_display_tab()

    def _refresh_sequence_pump_status(self):
        state = self.node.get_pump_on_state()
        if state is None:
            _set_status(self.sequence_pump_status_label, 'ポンプ: 不明(hand_node未起動?)', 'muted')
        elif state:
            _set_status(self.sequence_pump_status_label, 'ポンプ: ON(吸着中)', 'success')
        else:
            _set_status(self.sequence_pump_status_label, 'ポンプ: OFF', 'muted')

    def _refresh_current_state(self):
        if not self.node.has_current_state():
            return
        pos = self.node.get_current_positions()
        theta = pos['root_theta_joint']
        zj = pos['z_joint']
        r = pos['r_joint']
        x, y, z = joint_to_xyz(theta, zj, r)
        self.current_label.setText(
            f'theta={math.degrees(theta):.1f}deg  z_joint={zj:.3f}  r_joint={r:.3f}\n'
            f'X={x:.3f}  Y={y:.3f}  Z={z:.3f}')
        self.xy_widget.set_current(x, y)
        self.field_minimap.set_current(x, y)
        self.z_gauge.set_z(z)
        if not self._target_synced_to_current_:
            set_float(self.x_edit, round(x, 3))
            set_float(self.y_edit, round(y, 3))
            set_float(self.z_edit, round(z, 3))
            self._target_synced_to_current_ = True

    def _on_load_traj_params(self):
        # joint_namesも取得する: trajectory_follower_nodeは実行構成によって
        # JOINT_NAMES(root_theta/z/r)の全部ではなく一部だけで起動されることがある
        # (例: real_root_theta_test.launch.pyはroot_theta_jointのみ)。max_velocity等の
        # 配列は「そのノードの実際のjoint_names順」なので、GUI固定のJOINT_NAMESを
        # 前提にlen比較・zipすると、一致しない構成では表示が更新されず0のままに見える。
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME,
            ['max_velocity', 'max_acceleration', 'max_deceleration', 'control_mode', 'joint_names'],
            self._apply_loaded_traj_params,
            lambda reason: _set_status(self.traj_status_label, f'読込失敗: {reason}', 'error'))
        _set_status(self.traj_status_label,
                    '読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')
        return ok

    def _try_auto_setup_gains(self):
        # 軌道生成・MIT・robomas・joy速度それぞれについて、対象ノードのサービスが
        # 使えるようになり次第、自動で一度だけ「読込」→「適用」する(以後は明示的に
        # ボタンを押すまで再取得・再送信しない。編集中の値を上書きし続けない
        # ようにするため、カテゴリごとに一度成功したら止める)。
        # 「読込」はjoint_names等ノードの実際の構成を知るための表示同期のみ
        # (trajectory_follower_nodeの軌道生成読込には元々、real_all_axes_test.
        # launch.py等tip_theta_jointを含む4関節構成で「読込」未実行のままGUI固定の
        # 3関節(JOINT_NAMES)で「適用」すると「4要素必要」と拒否される問題を防ぐ
        # 意味もあった。_collect_traj_values/__init__のコメント参照)。
        # 「適用」はgains.json(_saved_gains、起動時にGUIへ復元済み)の値を実機へ
        # SetParametersする。手動で「適用」を押さない限りゲインが反映されず
        # robomas(z/r)が動かない、という問題(2026-09-02報告)への対処。
        if not self._traj_auto_loaded and self._on_load_traj_params():
            self._traj_auto_loaded = True
        if not self._mit_auto_loaded and self._on_load_mit_gains():
            self._mit_auto_loaded = True
        if not self._robomas_auto_loaded and self._on_load_robomas_gains():
            self._robomas_auto_loaded = True
        if not self._robomas_vel_auto_loaded and self._on_load_robomas_vel_gains():
            self._robomas_vel_auto_loaded = True
        if not self._joy_auto_loaded and self._on_load_joy_speed():
            self._joy_auto_loaded = True

        # 軌道生成・MITは配列の並び順・要素数がjoint_names次第のため、読込の
        # 「応答」が届いてから適用する(_traj_names_known/_mit_names_known参照。
        # 読込の「リクエスト送信済み」フラグ_traj_auto_loaded/_mit_auto_loadedは
        # 非同期の応答到着を待たないため、これで判定すると_traj_joint_names等が
        # まだ古い(GUI既定の)関節数のまま適用してしまい「4要素必要」等で
        # 失敗する。2026-09-02報告・修正)。robomas・joy速度はスカラー
        # パラメータのみで並び順の懸念がないため、読込を待たずに直接適用できる。
        if self._traj_names_known and not self._traj_auto_applied and self._auto_apply_saved_traj():
            self._traj_auto_applied = True
        if self._mit_names_known and not self._mit_auto_applied and self._auto_apply_saved_mit():
            self._mit_auto_applied = True
        if not self._robomas_auto_applied and self._auto_apply_saved_robomas():
            self._robomas_auto_applied = True
        if not self._robomas_vel_auto_applied and self._auto_apply_saved_robomas_vel():
            self._robomas_vel_auto_applied = True
        if not self._joy_auto_applied and self._auto_apply_saved_joy():
            self._joy_auto_applied = True

        if all((self._traj_auto_loaded, self._mit_auto_loaded,
                self._robomas_auto_loaded, self._robomas_vel_auto_loaded, self._joy_auto_loaded,
                self._traj_auto_applied, self._mit_auto_applied,
                self._robomas_auto_applied, self._robomas_vel_auto_applied, self._joy_auto_applied)):
            self._auto_load_timer.stop()

    def _auto_apply_saved_traj(self):
        """gains.jsonのtrajectory値を、読込で学習した_traj_joint_names順の配列に
        組み立てて自動適用する。未保存の関節がある場合はfalseを返し、次回tick
        (通常は_apply_loaded_traj_paramsが表示を更新した直後)に再試行する。"""
        saved = self._saved_gains.get('trajectory')
        if not saved:
            return True
        names = self._traj_joint_names
        vel_map = saved.get('max_velocity', {})
        accel_map = saved.get('max_acceleration', {})
        # max_decelerationはこのGUIの2026-09-07追加より前に保存されたgains.jsonには
        # 存在しない。無ければmax_accelerationの2倍(trap_step/move_timeの既定と
        # 同じ考え方)を使う(古いgains.jsonでも「4要素必要」等で自動適用が失敗し
        # 続けないようにするため)。
        decel_map = saved.get('max_deceleration', {})
        if not all(n in vel_map for n in names) or not all(n in accel_map for n in names):
            return False
        payload = {
            'max_velocity': [vel_map[n] for n in names],
            'max_acceleration': [accel_map[n] for n in names],
            'max_deceleration': [decel_map[n] if n in decel_map else accel_map[n] * 2.0 for n in names],
        }
        _set_status(self.traj_status_label, '自動適用中(gains.json)...', 'muted')
        return self.node.set_node_params(TRAJ_NODE_NAME, payload, self._apply_traj_set_result)

    def _auto_apply_saved_mit(self):
        saved = self._saved_gains.get('mit_gain')
        names = self._mit_joint_names
        if not saved or not names:
            return True
        kp_map = saved.get('cubemars_kp', {})
        kd_map = saved.get('cubemars_kd', {})
        tff_map = saved.get('cubemars_torque_ff', {})
        if not all(n in kp_map for n in names) or not all(n in kd_map for n in names):
            return False
        payload = {
            'cubemars_kp': [kp_map[n] for n in names],
            'cubemars_kd': [kd_map[n] for n in names],
            'cubemars_torque_ff': [tff_map.get(n, 0.0) for n in names],
        }
        _set_status(self.mit_gain_status_label, '自動適用中(gains.json)...', 'muted')
        return self.node.set_node_params(TRAJ_NODE_NAME, payload, self._apply_mit_gain_set_result)

    def _auto_apply_saved_robomas(self):
        saved = self._saved_gains.get('robomas_gain')
        if not saved:
            return True
        _set_status(self.robomas_gain_status_label, '自動適用中(gains.json)...', 'muted')
        return self.node.set_node_params(TRAJ_NODE_NAME, saved, self._apply_robomas_gain_set_result)

    def _auto_apply_saved_robomas_vel(self):
        saved = self._saved_gains.get('robomas_vel_gain')
        if not saved:
            return True
        _set_status(self.robomas_vel_gain_status_label, '自動適用中(gains.json)...', 'muted')
        return self.node.set_node_params(TRAJ_NODE_NAME, saved, self._apply_robomas_vel_gain_set_result)

    def _auto_apply_saved_joy(self):
        saved = self._saved_gains.get('joy_speed')
        if not saved:
            return True
        _set_status(self.joy_speed_status_label, '自動適用中(gains.json)...', 'muted')
        return self.node.set_node_params(JOY_NODE_NAME, saved, self._apply_joy_speed_set_result)

    def _apply_loaded_traj_params(self, values):
        vel = values.get('max_velocity')
        accel = values.get('max_acceleration')
        decel = values.get('max_deceleration')
        mode = values.get('control_mode')
        names = values.get('joint_names') or JOINT_NAMES
        self._traj_joint_names = list(names)
        # request_node_paramsは非同期(応答はrclpy.spin_once経由でこのコールバックが
        # 呼ばれて初めて届く)。_try_auto_setup_gainsはこのフラグを見てから
        # _auto_apply_saved_trajを呼ぶことで、_traj_joint_namesが実際の構成に
        # 更新される前(読込リクエストを送っただけの段階)に古い関節数のまま
        # 適用してしまい「4要素必要」等で失敗する問題を防ぐ(2026-09-02修正)。
        self._traj_names_known = True
        missing = [n for n in TRAJ_PANEL_JOINT_NAMES if n not in names]
        self._traj_loaded_extra = {}
        if vel and len(vel) == len(names):
            for name, v in zip(names, vel):
                if name in self.traj_vel_edits:
                    set_float(self.traj_vel_edits[name], round(v, 4))
                else:
                    self._traj_loaded_extra.setdefault(name, {})['max_velocity'] = v
        if accel and len(accel) == len(names):
            for name, v in zip(names, accel):
                if name in self.traj_accel_edits:
                    set_float(self.traj_accel_edits[name], round(v, 4))
                else:
                    self._traj_loaded_extra.setdefault(name, {})['max_acceleration'] = v
        if decel and len(decel) == len(names):
            for name, v in zip(names, decel):
                if name in self.traj_decel_edits:
                    set_float(self.traj_decel_edits[name], round(v, 4))
                else:
                    self._traj_loaded_extra.setdefault(name, {})['max_deceleration'] = v
        if mode:
            self._set_mode_silent(mode)
            _set_status(self.mode_status_label, f'現在のモード: {mode}', 'info')
        # 「読込」はノードの現在値をGUIに表示するだけに留め、gains.jsonへは
        # 「適用」時のみ永続化する(読込のたびに永続化すると、起動直後の
        # 自動読込(_try_auto_load_traj_params)やセットアップ手順での読込操作で
        # launchファイルの初期値がGUI保持の調整済み値を上書きしてしまう
        # バグになるため、2026-09-02に修正)。
        status = '読込完了'
        if missing:
            status += f' (未起動構成: {", ".join(missing)}は表示更新されません)'
        _set_status(self.traj_status_label, status, 'info')

    def _collect_traj_values(self):
        """軌道生成パラメータをGUI入力欄から取得する(self._traj_joint_names順)。
        「読込」未実行でもGUI固定のTRAJ_PANEL_JOINT_NAMES(root_theta/z/r/tip_theta)を
        対象に動作する(__init__参照)。万一これ以外の関節がノード側のjoint_names
        に含まれる場合は、_traj_loaded_extraに保持した読込時の値をそのまま使い、
        その関節の設定値を変更せず送り直す(real_all_axes_test.launch.py参照。
        2026-09-03、tip_theta_jointは編集欄を持つようになったためこの経路を
        通らなくなった。回収シーケンスがroot_theta_jointと同時にtip_theta_jointも
        指令するようになり、trajectory_follower_nodeの同時到達スケーリングにより
        tip_theta_jointの低いmax_velocity/max_accelerationがroot_theta_jointの
        実効速度まで引きずり下げる問題が起きたため、GUIから調整できるようにした)。"""
        names = self._traj_joint_names
        vel = [
            get_float(self.traj_vel_edits[name]) if name in self.traj_vel_edits
            else self._traj_loaded_extra[name]['max_velocity']
            for name in names
        ]
        accel = [
            get_float(self.traj_accel_edits[name]) if name in self.traj_accel_edits
            else self._traj_loaded_extra[name]['max_acceleration']
            for name in names
        ]
        decel = [
            get_float(self.traj_decel_edits[name]) if name in self.traj_decel_edits
            else self._traj_loaded_extra[name]['max_deceleration']
            for name in names
        ]
        return {'max_velocity': vel, 'max_acceleration': accel, 'max_deceleration': decel}

    def _on_apply_traj_params(self):
        try:
            traj = self._collect_traj_values()
        except (ValueError, KeyError):
            QMessageBox.critical(self, '入力エラー', '速度・加速度に数値を入力してください')
            return
        self._persist_traj_values(traj)
        ok = self.node.set_node_params(TRAJ_NODE_NAME, traj, self._apply_traj_set_result)
        _set_status(self.traj_status_label,
                    '適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')

    def _apply_traj_set_result(self, results):
        if results is None:
            _set_status(self.traj_status_label, '適用に失敗しました(応答なし)', 'error')
            return
        if all(r.successful for r in results):
            _set_status(self.traj_status_label, '適用しました', 'success')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.traj_status_label, f'適用失敗: {reasons}', 'error')

    def _on_load_mit_gains(self):
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME,
            ['cubemars_kp', 'cubemars_kd', 'cubemars_torque_ff', 'cubemars_joint_names'],
            self._apply_loaded_mit_gains,
            lambda reason: _set_status(self.mit_gain_status_label, f'読込失敗: {reason}', 'error'))
        _set_status(self.mit_gain_status_label,
                    '読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')
        return ok

    def _apply_loaded_mit_gains(self, values):
        kp = values.get('cubemars_kp')
        kd = values.get('cubemars_kd')
        tff = values.get('cubemars_torque_ff')
        names = values.get('cubemars_joint_names') or []
        self._mit_joint_names = list(names)
        # _traj_names_knownと同じ理由(_apply_loaded_traj_paramsのコメント参照)。
        self._mit_names_known = True
        if kp and len(kp) == len(names):
            for name, v in zip(names, kp):
                if name in self.mit_kp_edits:
                    set_float(self.mit_kp_edits[name], round(v, 4))
        if kd and len(kd) == len(names):
            for name, v in zip(names, kd):
                if name in self.mit_kd_edits:
                    set_float(self.mit_kd_edits[name], round(v, 4))
        if tff and len(tff) == len(names):
            for name, v in zip(names, tff):
                if name in self.mit_torque_edits:
                    set_float(self.mit_torque_edits[name], round(v, 4))
        # 「読込」はノードの現在値をGUIに表示するだけに留め、gains.jsonへは
        # 「適用」時のみ永続化する(_apply_loaded_traj_paramsのコメント参照)。
        status = '読込完了'
        not_configured = [n for n in CUBEMARS_JOINT_NAMES if n not in names]
        if not_configured:
            status += f' (実機出力対象外: {", ".join(not_configured)})'
        if not names:
            status = '読込完了 (実機出力対象の関節が設定されていません)'
        _set_status(self.mit_gain_status_label, status, 'info')

    def _collect_mit_values(self):
        """MITゲインをGUI入力欄から取得する(self._mit_joint_names順)。「読込」
        未実行の場合はCUBEMARS_JOINT_NAMES(GUI固定の対象関節)を用いる(__init__参照)。
        読込済みでノードが対象関節0件を報告した場合はself._mit_joint_names==[]と
        なり、長さ0の配列(=実質何もしない)を返す。"""
        names = self._mit_joint_names
        kp = [get_float(self.mit_kp_edits[name]) for name in names]
        kd = [get_float(self.mit_kd_edits[name]) for name in names]
        tff = [get_float(self.mit_torque_edits[name]) for name in names]
        return {'cubemars_kp': kp, 'cubemars_kd': kd, 'cubemars_torque_ff': tff}

    def _on_apply_mit_gains(self):
        names = self._mit_joint_names
        try:
            mit = self._collect_mit_values()
        except (ValueError, KeyError):
            QMessageBox.critical(self, '入力エラー', 'Kp/Kd/torque_ffに数値を入力してください')
            return
        if not names:
            _set_status(self.mit_gain_status_label,
                        '実機出力対象の関節が設定されていないため送信をスキップしました', 'muted')
            return
        reply = QMessageBox.question(
            self, 'MITゲイン適用の確認',
            f'{", ".join(names)} のMITゲインを実機へ即座に反映します。\n'
            'Kpを大きくするほど保持力・応答性が上がりますが、\n'
            '実機にかかる力も大きくなります。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self._persist_mit_values(mit)
        ok = self.node.set_node_params(TRAJ_NODE_NAME, mit, self._apply_mit_gain_set_result)
        _set_status(self.mit_gain_status_label,
                    '適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')

    def _apply_mit_gain_set_result(self, results):
        if results is None:
            _set_status(self.mit_gain_status_label, '適用に失敗しました(応答なし)', 'error')
            return
        if all(r.successful for r in results):
            _set_status(self.mit_gain_status_label, '適用しました', 'success')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.mit_gain_status_label, f'適用失敗: {reasons}', 'error')

    def _build_robomas_gain_panel(self, column):
        # robomas_kp/kd/current_ffはcubemars_*と異なりz_joint/r_joint(motor1/motor2)
        # 共通のスカラー値(joint別ではない、trajectory_follower_node.py参照)なので、
        # MITゲインパネルのような関節ごとの行ではなく単一行で表示する。
        box = QGroupBox('MITゲイン (実機ロボマス、trajectory_follower_node)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, 'motor1/motor2(z/r)共通の値。robomas_device_id未設定なら'
                          '実機出力無効。', 'muted')
        layout.addWidget(desc)

        grid = QGridLayout()
        self.robomas_kp_edit = make_float_edit(0.0, width=70)
        self.robomas_kd_edit = make_float_edit(0.0, width=70)
        self.robomas_current_ff_edit = make_float_edit(0.0, width=70)
        for i, (label, edit) in enumerate((
                ('Kp [A/deg]', self.robomas_kp_edit),
                ('Kd [A/rpm]', self.robomas_kd_edit),
                ('current_ff [A]', self.robomas_current_ff_edit))):
            grid.addWidget(QLabel(label), i, 0)
            grid.addWidget(edit, i, 1)
        layout.addLayout(grid)

        # 動き出しキック(静止摩擦補償、2026-09-09追加。trajectory_follower_node.pyの
        # robomas_kick_*パラメータ宣言部コメント参照)。current_ffと違い移動方向に
        # 応じて符号が変わるため、逆方向の動きを阻害しない。motor1/motor2独立に
        # 「止まっていた状態から動き出した」瞬間を検出し、一定時間だけ電流を上乗せする。
        kick_box = QGroupBox('動き出しキック (静止摩擦補償)')
        kick_layout = QVBoxLayout(kick_box)
        kick_desc = QLabel()
        kick_desc.setWordWrap(True)
        _set_status(
            kick_desc,
            '差動の片方だけ動き出しのトルクが足りない場合用。current_ffと異なり\n'
            '移動方向に応じて符号が変わるため、逆方向の動きを阻害しない\n'
            '(motor1/motor2それぞれ独立に、停止->動き出しの瞬間だけ働く)。',
            'muted')
        kick_layout.addWidget(kick_desc)

        self.robomas_kick_enabled_check = QCheckBox('有効化')
        kick_layout.addWidget(self.robomas_kick_enabled_check)

        kick_grid = QGridLayout()
        self.robomas_kick_current_a_edit = make_float_edit(0.1, width=70)
        self.robomas_kick_duration_sec_edit = make_float_edit(0.05, width=70)
        self.robomas_kick_vel_threshold_mps_edit = make_float_edit(0.001, width=70)
        for i, (label, edit) in enumerate((
                ('キック電流 [A]', self.robomas_kick_current_a_edit),
                ('持続時間 [s]', self.robomas_kick_duration_sec_edit),
                ('動作判定しきい値 [m/s]', self.robomas_kick_vel_threshold_mps_edit))):
            kick_grid.addWidget(QLabel(label), i, 0)
            kick_grid.addWidget(edit, i, 1)
        kick_layout.addLayout(kick_grid)
        layout.addWidget(kick_box)

        # tip_theta(M3、2026-09-08新規)はz/r(motor1/motor2)と動特性が異なるため
        # 別ゲインを持つ。同じdeviceに同居するため読込/適用ボタンは共通のまま、
        # 入力欄だけ分ける。
        layout.addWidget(QLabel('tip_theta(M3)'))
        tip_grid = QGridLayout()
        self.robomas_tip_theta_kp_edit = make_float_edit(0.0, width=70)
        self.robomas_tip_theta_kd_edit = make_float_edit(0.0, width=70)
        self.robomas_tip_theta_current_ff_edit = make_float_edit(0.0, width=70)
        for i, (label, edit) in enumerate((
                ('Kp [A/deg]', self.robomas_tip_theta_kp_edit),
                ('Kd [A/rpm]', self.robomas_tip_theta_kd_edit),
                ('current_ff [A]', self.robomas_tip_theta_current_ff_edit))):
            tip_grid.addWidget(QLabel(label), i, 0)
            tip_grid.addWidget(edit, i, 1)
        layout.addLayout(tip_grid)

        self.robomas_gain_status_label = QLabel()
        self.robomas_gain_status_label.setWordWrap(True)
        _set_status(self.robomas_gain_status_label, '未読込', 'muted')
        layout.addWidget(self.robomas_gain_status_label)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'danger')
        load_btn.clicked.connect(self._on_load_robomas_gains)
        apply_btn.clicked.connect(self._on_apply_robomas_gains)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        layout.addLayout(btn_row)

        # homing_node実行中はtrajectory_follower_node側が自動でpause/resumeするが
        # (note/hardware_mapping.txt参照)、実機調整時に手動で止めたい場合用に
        # root_theta原点パネルと同じTriggerボタンの型でも操作できるようにする。
        pause_row = QHBoxLayout()
        pause_btn = QPushButton('出力を一時停止')
        resume_btn = QPushButton('出力を再開')
        pause_btn.clicked.connect(lambda: self._on_robomas_pause_resume('/pause_robomas_output'))
        resume_btn.clicked.connect(lambda: self._on_robomas_pause_resume('/resume_robomas_output'))
        pause_row.addWidget(pause_btn)
        pause_row.addWidget(resume_btn)
        layout.addLayout(pause_row)

        column.addWidget(box)

    def _on_load_robomas_gains(self):
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME,
            ['robomas_kp', 'robomas_kd', 'robomas_current_ff',
             'robomas_kick_enabled', 'robomas_kick_current_a',
             'robomas_kick_duration_sec', 'robomas_kick_vel_threshold_mps',
             'robomas_tip_theta_kp', 'robomas_tip_theta_kd', 'robomas_tip_theta_current_ff',
             'robomas_device_id'],
            self._apply_loaded_robomas_gains,
            lambda reason: _set_status(self.robomas_gain_status_label, f'読込失敗: {reason}', 'error'))
        _set_status(self.robomas_gain_status_label,
                    '読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')
        return ok

    def _apply_loaded_robomas_gains(self, values):
        if 'robomas_kp' in values:
            set_float(self.robomas_kp_edit, round(values['robomas_kp'], 6))
        if 'robomas_kd' in values:
            set_float(self.robomas_kd_edit, round(values['robomas_kd'], 6))
        if 'robomas_current_ff' in values:
            set_float(self.robomas_current_ff_edit, round(values['robomas_current_ff'], 6))
        if 'robomas_kick_enabled' in values:
            self.robomas_kick_enabled_check.setChecked(bool(values['robomas_kick_enabled']))
        if 'robomas_kick_current_a' in values:
            set_float(self.robomas_kick_current_a_edit, round(values['robomas_kick_current_a'], 6))
        if 'robomas_kick_duration_sec' in values:
            set_float(self.robomas_kick_duration_sec_edit, round(values['robomas_kick_duration_sec'], 6))
        if 'robomas_kick_vel_threshold_mps' in values:
            set_float(self.robomas_kick_vel_threshold_mps_edit,
                      round(values['robomas_kick_vel_threshold_mps'], 6))
        if 'robomas_tip_theta_kp' in values:
            set_float(self.robomas_tip_theta_kp_edit, round(values['robomas_tip_theta_kp'], 6))
        if 'robomas_tip_theta_kd' in values:
            set_float(self.robomas_tip_theta_kd_edit, round(values['robomas_tip_theta_kd'], 6))
        if 'robomas_tip_theta_current_ff' in values:
            set_float(self.robomas_tip_theta_current_ff_edit,
                      round(values['robomas_tip_theta_current_ff'], 6))
        # 「読込」はノードの現在値をGUIに表示するだけに留め、gains.jsonへは
        # 「適用」時のみ永続化する(_apply_loaded_traj_paramsのコメント参照)。
        device_id = values.get('robomas_device_id', 0)
        status = '読込完了'
        if not device_id:
            status += ' (実機出力無効: robomas_device_id未設定)'
        else:
            status += f' (device_id={device_id})'
        _set_status(self.robomas_gain_status_label, status, 'info')

    def _collect_robomas_values(self):
        return {
            'robomas_kp': get_float(self.robomas_kp_edit),
            'robomas_kd': get_float(self.robomas_kd_edit),
            'robomas_current_ff': get_float(self.robomas_current_ff_edit),
            'robomas_kick_enabled': self.robomas_kick_enabled_check.isChecked(),
            'robomas_kick_current_a': get_float(self.robomas_kick_current_a_edit),
            'robomas_kick_duration_sec': get_float(self.robomas_kick_duration_sec_edit),
            'robomas_kick_vel_threshold_mps': get_float(self.robomas_kick_vel_threshold_mps_edit),
            'robomas_tip_theta_kp': get_float(self.robomas_tip_theta_kp_edit),
            'robomas_tip_theta_kd': get_float(self.robomas_tip_theta_kd_edit),
            'robomas_tip_theta_current_ff': get_float(self.robomas_tip_theta_current_ff_edit),
        }

    def _on_apply_robomas_gains(self):
        try:
            values = self._collect_robomas_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'Kp/Kd/current_ffに数値を入力してください')
            return
        reply = QMessageBox.question(
            self, 'MITゲイン適用の確認',
            'motor1/motor2(z/r)・tip_theta(M3)のMITゲインを実機へ即座に反映します。\n'
            'Kpを大きくするほど保持力・応答性が上がりますが、\n'
            '実機にかかる力も大きくなります。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self._persist_gains('robomas_gain', values)
        ok = self.node.set_node_params(TRAJ_NODE_NAME, values, self._apply_robomas_gain_set_result)
        _set_status(self.robomas_gain_status_label,
                    '適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')

    def _apply_robomas_gain_set_result(self, results):
        if results is None:
            _set_status(self.robomas_gain_status_label, '適用に失敗しました(応答なし)', 'error')
            return
        if all(r.successful for r in results):
            _set_status(self.robomas_gain_status_label, '適用しました', 'success')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.robomas_gain_status_label, f'適用失敗: {reasons}', 'error')

    def _on_robomas_pause_resume(self, service_name):
        _set_status(self.robomas_gain_status_label, f'{service_name} 呼び出し中...', 'muted')
        ok = self.node.call_trigger_service(
            service_name, self._on_robomas_pause_resume_done)
        if not ok:
            _set_status(self.robomas_gain_status_label, 'サービス未起動です', 'error')

    def _on_robomas_pause_resume_done(self, success, message):
        _set_status(self.robomas_gain_status_label, message, 'success' if success else 'error')

    def _build_robomas_autotune_panel(self, column):
        # z/rのMITゲイン(robomas_kp/kd)をautotune_nodeで自動探索するパネル
        # (2026-09-09追加)。motor1/motor2共通ゲインという実機構成上、z軸試験・
        # r軸試験を別々に実行できるが提案されるゲインは両軸共通の1組になる点は
        # 上のMITゲインパネルと同じ(_build_robomas_gain_panelのコメント参照)。
        box = QGroupBox('MITゲイン 自動調整 (Z/R別、autotune_node)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(
            desc,
            '現在位置を中心に前後(既定±2cm)へ往復させながらkp/kdを自動探索する。\n'
            'z軸試験・r軸試験は別々に実行できるが、実機がM1/M2共通ゲインのため\n'
            '提案されるkp/kdは常に両軸で共通の1組になる。\n'
            '開始前にリミットスイッチから十分離れた位置に置くこと。\n'
            '完了・中断のいずれでも実機ゲインは自動で元の値へ戻る(提案値は\n'
            '下の「読込」で表示させてから内容を確認し、必要ならkp/kd欄を書き換えて\n'
            '「適用」で反映すること。振幅・試行回数等はros2 param(autotune_node)で調整可)。',
            'muted')
        layout.addWidget(desc)

        self.autotune_status_label = QLabel()
        self.autotune_status_label.setWordWrap(True)
        _set_status(self.autotune_status_label, '未実行', 'muted')
        layout.addWidget(self.autotune_status_label)

        self.autotune_progress_label = QLabel()
        self.autotune_progress_label.setWordWrap(True)
        self.autotune_progress_label.setStyleSheet('font-size: 10px;')
        _set_status(self.autotune_progress_label, '', 'muted')
        layout.addWidget(self.autotune_progress_label)

        start_row = QHBoxLayout()
        start_z_btn = QPushButton('z軸 自動調整開始')
        start_z_btn.setProperty('variant', 'primary')
        start_r_btn = QPushButton('r軸 自動調整開始')
        start_r_btn.setProperty('variant', 'primary')
        start_z_btn.clicked.connect(self._on_start_autotune_z)
        start_r_btn.clicked.connect(self._on_start_autotune_r)
        start_row.addWidget(start_z_btn)
        start_row.addWidget(start_r_btn)
        layout.addLayout(start_row)

        stop_btn = QPushButton('中断')
        stop_btn.setProperty('variant', 'danger')
        stop_btn.clicked.connect(self._on_stop_autotune)
        layout.addWidget(stop_btn)

        self._autotune_progress_shown = None  # 直近に表示した進捗文字列(重複更新防止)
        column.addWidget(box)

    def _on_start_autotune_axis(self, axis_label, service_name):
        reply = QMessageBox.question(
            self, f'{axis_label}自動ゲイン調整開始の確認',
            f'現在位置を中心に前後へ{axis_label}を振幅ぶん動かしながらkp/kdを探索します\n'
            '(数分かかります)。リミットスイッチから十分離れた位置にあるか確認してください。\n'
            '探索中はkp/kdが実機へ一時的に反映されますが、完了・中断時に元の値へ戻ります。',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        _set_status(self.autotune_status_label, f'{axis_label}: 開始中...', 'muted')
        ok = self.node.call_trigger_service(service_name, self._on_autotune_service_done)
        if not ok:
            _set_status(self.autotune_status_label, 'サービス未起動です(autotune_node起動確認)', 'error')

    def _on_start_autotune_z(self):
        self._on_start_autotune_axis('z軸', '/start_autotune_z')

    def _on_start_autotune_r(self):
        self._on_start_autotune_axis('r軸', '/start_autotune_r')

    def _on_stop_autotune(self):
        _set_status(self.autotune_status_label, '中断中...', 'muted')
        ok = self.node.call_trigger_service('/stop_autotune', self._on_autotune_service_done)
        if not ok:
            _set_status(self.autotune_status_label, 'サービス未起動です(autotune_node起動確認)', 'error')

    def _on_autotune_service_done(self, success, message):
        _set_status(self.autotune_status_label, message, 'success' if success else 'error')

    def _refresh_autotune_progress(self):
        """autotune_progress(試行ごとのkp/kd/score)は文字列トピックのため、
        homing_statusと違いサービス応答ではなくポーリングで表示を更新する
        (_refresh_machine_statusから1秒間隔で呼ばれる)。"""
        if not hasattr(self, 'autotune_progress_label'):
            return
        progress = self.node.get_autotune_progress()
        if progress is not None and progress != self._autotune_progress_shown:
            self._autotune_progress_shown = progress
            _set_status(self.autotune_progress_label, progress, 'info')

    def _build_homing_panel(self, column):
        box = QGroupBox('z/rホーミング (homing_node)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, '開始(z軸/r軸): motor1/motor2を低速駆動しその軸の\n'
                          '原点センサまで動かす(z/rは独立に実行できるが、差動\n'
                          '機構のためどちらの軸でもmotor1/motor2両方が回転する)。\n'
                          'スキップ: 機体を先に原点センサ位置相当へ手動で\n'
                          '合わせてから使うこと(モータは駆動せずz/r両方を反映)。', 'muted')
        layout.addWidget(desc)

        self.homing_status_label = QLabel()
        self.homing_status_label.setWordWrap(True)
        _set_status(self.homing_status_label, '未実行', 'muted')
        layout.addWidget(self.homing_status_label)

        start_row = QHBoxLayout()
        start_z_btn = QPushButton('z軸 開始')
        start_z_btn.setProperty('variant', 'primary')
        start_r_btn = QPushButton('r軸 開始')
        start_r_btn.setProperty('variant', 'primary')
        start_z_btn.clicked.connect(self._on_start_homing_z)
        start_r_btn.clicked.connect(self._on_start_homing_r)
        start_row.addWidget(start_z_btn)
        start_row.addWidget(start_r_btn)
        layout.addLayout(start_row)

        btn_row = QHBoxLayout()
        stop_btn = QPushButton('中断')
        skip_btn = QPushButton('スキップ(z/r両方)')
        skip_btn.setProperty('variant', 'danger')
        stop_btn.clicked.connect(self._on_stop_homing)
        skip_btn.clicked.connect(self._on_skip_homing)
        btn_row.addWidget(stop_btn)
        btn_row.addWidget(skip_btn)
        layout.addLayout(btn_row)

        column.addWidget(box)

    def _on_start_homing_axis(self, axis_label, service_name):
        reply = QMessageBox.question(
            self, f'{axis_label}ホーミング開始の確認',
            f'motor1/motor2を低速駆動して{axis_label}原点センサまで動かします\n'
            f'(差動機構のためmotor1/motor2両方が回転します)。\n'
            '周囲に人・障害物がないか確認してください。',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        _set_status(self.homing_status_label, f'{axis_label}: 開始中...', 'muted')
        ok = self.node.call_trigger_service(service_name, self._on_homing_service_done)
        if not ok:
            _set_status(self.homing_status_label, 'サービス未起動です(homing_node起動確認)', 'error')

    def _on_start_homing_z(self):
        self._on_start_homing_axis('z軸', '/start_homing_z')

    def _on_start_homing_r(self):
        self._on_start_homing_axis('r軸', '/start_homing_r')

    def _on_stop_homing(self):
        _set_status(self.homing_status_label, '中断中...', 'muted')
        ok = self.node.call_trigger_service('/stop_homing', self._on_homing_service_done)
        if not ok:
            _set_status(self.homing_status_label, 'サービス未起動です(homing_node起動確認)', 'error')

    def _on_skip_homing(self):
        reply = QMessageBox.question(
            self, 'ホーミングスキップの確認',
            'motor1/motor2は駆動せず、現在位置を原点センサ位置(z_ref_value_m/\n'
            'r_ref_value_m)とみなしてoffsetを即座に反映します。\n\n'
            '機体は今、原点センサ位置相当にありますか？\n'
            '間違った位置で実行すると以後のz/r値が全てズレます。',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        _set_status(self.homing_status_label, 'スキップ処理中...', 'muted')
        ok = self.node.call_trigger_service('/skip_homing', self._on_homing_service_done)
        if not ok:
            _set_status(self.homing_status_label, 'サービス未起動です(homing_node起動確認)', 'error')

    def _on_homing_service_done(self, success, message):
        _set_status(self.homing_status_label, message, 'success' if success else 'error')

    def _on_load_joy_speed(self):
        ok = self.node.request_node_params(
            JOY_NODE_NAME, list(self.joy_speed_edits.keys()),
            self._apply_loaded_joy_speed,
            lambda reason: _set_status(self.joy_speed_status_label, f'読込失敗: {reason}', 'error'))
        _set_status(self.joy_speed_status_label,
                    '読込中...' if ok else 'joy_teleop_nodeに接続できません(use_joy:=trueで起動?)',
                    'muted' if ok else 'error')
        return ok

    def _apply_loaded_joy_speed(self, values):
        for name in self.joy_speed_edits:
            if name in values:
                set_float(self.joy_speed_edits[name], round(values[name], 4))
        # 「読込」はノードの現在値をGUIに表示するだけに留め、gains.jsonへは
        # 「適用」時のみ永続化する(_apply_loaded_traj_paramsのコメント参照)。
        _set_status(self.joy_speed_status_label, '読込完了', 'info')

    def _collect_joy_speed_values(self):
        return {name: get_float(edit) for name, edit in self.joy_speed_edits.items()}

    def _on_apply_joy_speed(self):
        try:
            values = self._collect_joy_speed_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', '速度に数値を入力してください')
            return
        self._persist_gains('joy_speed', values)
        ok = self.node.set_node_params(JOY_NODE_NAME, values, self._apply_joy_speed_set_result)
        if 'low_speed_multiplier' in values:
            # trajectory_follower_node側も同じ倍率を使う(low_speed_multiplier
            # フィールドのコメント参照)。joy_teleop_node宛の主リクエストとは別に
            # best-effortで送るだけで、失敗してもjoy_speed_status_labelは主
            # リクエスト側の結果を表示する。
            self.node.set_node_params(
                TRAJ_NODE_NAME, {'low_speed_multiplier': values['low_speed_multiplier']})
        _set_status(self.joy_speed_status_label,
                    '適用中...' if ok else 'joy_teleop_nodeに接続できません(use_joy:=trueで起動?)',
                    'muted' if ok else 'error')

    def _apply_joy_speed_set_result(self, results):
        if results is None:
            _set_status(self.joy_speed_status_label, '適用に失敗しました(応答なし)', 'error')
            return
        if all(r.successful for r in results):
            _set_status(self.joy_speed_status_label, '適用しました', 'success')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            _set_status(self.joy_speed_status_label, f'適用失敗: {reasons}', 'error')

    def _on_mode_changed(self, mode):
        ok = self.node.set_node_params(TRAJ_NODE_NAME, {'control_mode': mode}, self._apply_mode_set_result)
        _set_status(self.mode_status_label,
                    '適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
                    'muted' if ok else 'error')

    def _apply_mode_set_result(self, results):
        if results is None or not all(r.successful for r in results):
            reason = results[0].reason if results else '応答なし'
            _set_status(self.mode_status_label, f'適用失敗: {reason}', 'error')
            return
        current = next((v for v, rb in self._mode_buttons.items() if rb.isChecked()), '?')
        _set_status(self.mode_status_label, f'現在のモード: {current}', 'success')

    def _on_copy_current_to_target(self):
        if not self.node.has_current_state():
            QMessageBox.information(self, '未取得', 'まだ現在位置を受信していません')
            return
        pos = self.node.get_current_positions()
        x, y, z = joint_to_xyz(pos['root_theta_joint'], pos['z_joint'], pos['r_joint'])
        set_float(self.x_edit, round(x, 3))
        set_float(self.y_edit, round(y, 3))
        set_float(self.z_edit, round(z, 3))

    # ---------- coordinate handling ----------
    def _on_canvas_click(self, x, y):
        radius = math.hypot(x, y)
        if radius > MAX_RADIUS:
            x *= MAX_RADIUS / radius
            y *= MAX_RADIUS / radius
        set_float(self.x_edit, round(x, 3))
        set_float(self.y_edit, round(y, 3))

    def _redraw_pin(self, *_args):
        try:
            x = get_float(self.x_edit)
            y = get_float(self.y_edit)
        except ValueError:
            return
        self.xy_widget.set_pin(x, y)
        self._update_status()

    def _update_status(self):
        try:
            x = get_float(self.x_edit)
            y = get_float(self.y_edit)
            z = get_float(self.z_edit)
        except ValueError:
            return
        theta, zj, r, clamped = xyz_to_joint(x, y, z)
        text = f'theta={math.degrees(theta):.1f}deg  z_joint={zj:.3f}  r_joint={r:.3f}'
        if clamped:
            text += '\n(可動域外のためクランプされました)'
        _set_status(self.status_label, text, 'error' if clamped else 'info')

    # ---------- send ----------
    def _on_send(self):
        try:
            x = get_float(self.x_edit)
            y = get_float(self.y_edit)
            z = get_float(self.z_edit)
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'X/Y/Zに数値を入力してください')
            return
        theta, zj, r, _ = xyz_to_joint(x, y, z)
        self.node.send_target(theta, zj, r)
        self._update_status()

    def _on_jog(self, dx, dy):
        try:
            step = get_float(self.step_edit)
            x = get_float(self.x_edit) + dx * step
            y = get_float(self.y_edit) + dy * step
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'X/Y/ステップに数値を入力してください')
            return
        set_float(self.x_edit, round(x, 4))
        set_float(self.y_edit, round(y, 4))
        self._on_send()

    def _send_xyz(self, x, y, z):
        set_float(self.x_edit, x)
        set_float(self.y_edit, y)
        set_float(self.z_edit, z)
        self._on_send()

    # ---------- points list ----------
    def _on_add_point(self):
        try:
            x = get_float(self.x_edit)
            y = get_float(self.y_edit)
            z = get_float(self.z_edit)
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'X/Y/Zに数値を入力してください')
            return
        name, ok = QInputDialog.getText(
            self, 'ポイント名', '保存する名前を入力してください',
            text=f'Point{len(self.points) + 1}')
        if not ok or not name:
            return
        self.points.append({'name': name, 'x': x, 'y': y, 'z': z})
        self._save_points()
        self._refresh_point_list()

    def _on_delete_point(self):
        row = self.listbox.currentRow()
        if row < 0:
            return
        del self.points[row]
        self._save_points()
        self._refresh_point_list()

    def _on_load_point(self):
        row = self.listbox.currentRow()
        if row < 0:
            return
        p = self.points[row]
        set_float(self.x_edit, p['x'])
        set_float(self.y_edit, p['y'])
        set_float(self.z_edit, p['z'])

    def _on_send_selected(self):
        row = self.listbox.currentRow()
        if row < 0:
            QMessageBox.information(self, '未選択', '一覧からポイントを選択してください')
            return
        p = self.points[row]
        self._send_xyz(p['x'], p['y'], p['z'])

    def _refresh_point_list(self):
        self.listbox.clear()
        for p in self.points:
            self.listbox.addItem(f"{p['name']}  ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f})")

    # ---------- persistence ----------
    @staticmethod
    def _load_points():
        if not os.path.exists(POINTS_FILE):
            return []
        try:
            with open(POINTS_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError):
            return []

    def _save_points(self):
        os.makedirs(os.path.dirname(POINTS_FILE), exist_ok=True)
        with open(POINTS_FILE, 'w', encoding='utf-8') as f:
            json.dump(self.points, f, ensure_ascii=False, indent=2)

    # ---------- gain persistence ----------
    # 軌道生成・MIT・robomas・joy速度の各ゲインは、「読込」(GetParameters)を
    # 経由しなくても起動直後からGUI保持値を使えるよう、最後に適用/読込した値を
    # gains.jsonへ保存し、起動時にGUI入力欄の初期値として復元する。ノード側の
    # yamlデフォルトとは独立した「GUIが最後に確認した値」のキャッシュである。
    # soki_sim/config/gains.json(git管理下、_resolve_gains_file_path参照)に
    # 保存することで、実機で有効だった値を失わずバックアップ・共有できる。
    @staticmethod
    def _load_gains_file():
        path = _resolve_gains_file_path()
        if not os.path.exists(path):
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError):
            return {}

    def _persist_gains(self, section, data):
        self._saved_gains[section] = data
        path = _resolve_gains_file_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self._saved_gains, f, ensure_ascii=False, indent=2)

    def _persist_traj_values(self, traj):
        names = self._traj_joint_names
        self._persist_gains('trajectory', {
            'max_velocity': dict(zip(names, traj['max_velocity'])),
            'max_acceleration': dict(zip(names, traj['max_acceleration'])),
            'max_deceleration': dict(zip(names, traj['max_deceleration'])),
        })

    def _persist_mit_values(self, mit):
        names = self._mit_joint_names
        self._persist_gains('mit_gain', {
            'cubemars_kp': dict(zip(names, mit['cubemars_kp'])),
            'cubemars_kd': dict(zip(names, mit['cubemars_kd'])),
            'cubemars_torque_ff': dict(zip(names, mit['cubemars_torque_ff'])),
        })

    def _restore_saved_gains(self):
        """__init__終盤(各パネル構築後)に一度だけ呼び、gains.jsonの内容を各
        入力欄の初期値へ反映する。ファイルが無い/壊れている場合は0.0のまま
        (make_float_editの初期値)とする。"""
        traj = self._saved_gains.get('trajectory', {})
        for key, edits in (('max_velocity', self.traj_vel_edits), ('max_acceleration', self.traj_accel_edits),
                           ('max_deceleration', self.traj_decel_edits)):
            for name, v in traj.get(key, {}).items():
                if name in edits:
                    set_float(edits[name], v)

        mit = self._saved_gains.get('mit_gain', {})
        for key, edits in (('cubemars_kp', self.mit_kp_edits), ('cubemars_kd', self.mit_kd_edits),
                           ('cubemars_torque_ff', self.mit_torque_edits)):
            for name, v in mit.get(key, {}).items():
                if name in edits:
                    set_float(edits[name], v)

        robomas = self._saved_gains.get('robomas_gain', {})
        if 'robomas_kp' in robomas:
            set_float(self.robomas_kp_edit, robomas['robomas_kp'])
        if 'robomas_kd' in robomas:
            set_float(self.robomas_kd_edit, robomas['robomas_kd'])
        if 'robomas_current_ff' in robomas:
            set_float(self.robomas_current_ff_edit, robomas['robomas_current_ff'])
        if 'robomas_kick_enabled' in robomas:
            self.robomas_kick_enabled_check.setChecked(bool(robomas['robomas_kick_enabled']))
        if 'robomas_kick_current_a' in robomas:
            set_float(self.robomas_kick_current_a_edit, robomas['robomas_kick_current_a'])
        if 'robomas_kick_duration_sec' in robomas:
            set_float(self.robomas_kick_duration_sec_edit, robomas['robomas_kick_duration_sec'])
        if 'robomas_kick_vel_threshold_mps' in robomas:
            set_float(self.robomas_kick_vel_threshold_mps_edit, robomas['robomas_kick_vel_threshold_mps'])

        joy = self._saved_gains.get('joy_speed', {})
        for name, edit in self.joy_speed_edits.items():
            if name in joy:
                set_float(edit, joy[name])

        sequence = self._saved_gains.get('sequence', {})
        for name, edit in self.sequence_edits.items():
            if name in sequence:
                set_float(edit, sequence[name])

    def _check_existing_launch_nodes(self):
        """GUI起動時に、自分が把握していない(self._launch_process=Noneのままの)
        実機ノード群が既に動いていないか確認する(ALL_AXES_LAUNCH_NODE_NAMES定義部の
        コメント参照)。前回のGUIセッションでcloseEventの「動かしたまま終了」を
        選んだか、GUIプロセス自体が異常終了してcloseEventが呼ばれなかった場合、
        start_new_session=Trueで独立させたlaunchプロセスは生き残り続けるが、
        self._launch_processはこのGUIプロセスのメモリ上の変数でしかないため
        新しいGUIプロセスからはその存在が見えず、「停止」ボタンでも管理できない。
        気づかず放置される事故を防ぐため、起動時に一度だけ警告する。"""
        try:
            running = set(self.node.get_node_names()) & ALL_AXES_LAUNCH_NODE_NAMES
        except Exception:
            return
        if not running:
            return
        QMessageBox.warning(
            self, '起動中の実機ノードを検知',
            'このGUIが起動していないはずの実機ノードが既に動作中です:\n'
            '  ' + ', '.join(sorted(running)) + '\n\n'
            'おそらく前回のGUIセッションで「全ノード起動」したプロセスが、\n'
            'GUI終了時に停止されずそのまま生き残っています\n'
            '(self._launch_processはGUIプロセスごとに独立した変数のため、\n'
            'このGUIの「停止」ボタンでは止められません)。\n\n'
            '意図しない場合は、ターミナルで以下を確認し、該当プロセスを\n'
            '終了させてください:\n'
            '  ros2 node list\n'
            '  pkill -INT -f real_all_axes_test.launch.py')

    def closeEvent(self, event):
        """実機セットアップパネルで起動したros2 launch子プロセスが残っている場合、
        GUI終了時に確認ダイアログなしで自動的に停止してから終了する(通常のCtrl+Cと
        同様のSIGINTで、ros2 launch側に配下ノードをまとめて終了させる)。

        2026-09-07、ユーザー指定:「デフォで閉じるときもダイヤログなしですべて停止して」
        により、以前あった「停止して終了/動かしたまま終了/キャンセル」の確認ダイアログを
        廃止した(start_new_session=Trueで独立させているlaunchプロセスが、GUI終了時に
        道連れにならず生き残り続けるインシデントが発生していたため、GUIを閉じたら
        常に停止する、を既定かつ唯一の動作にした)。"""
        if self._launch_process is not None and self._launch_process.poll() is None:
            self._signal_launch_process_group(signal.SIGINT)
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(_load_stylesheet())
    node = CommandGuiNode()
    window = CommandGuiApp(node)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
