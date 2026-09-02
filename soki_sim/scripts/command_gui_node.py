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
import json
import math
import os
import re
import signal
import subprocess
import sys
import unicodedata

import yaml

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QColor, QDoubleValidator, QFontMetrics, QIcon, QIntValidator, QPainter, QPen, QPixmap
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

JOINT_NAMES = ['root_theta_joint', 'z_joint', 'r_joint']
# MITゲインパネル(CubeMars)対象関節。JOINT_NAMESとは別物: root_theta/tip_thetaの
# 2つがCubeMars駆動(z/rはロボマス駆動、note/hardware_mapping.txt参照)。
CUBEMARS_JOINT_NAMES = ['root_theta_joint', 'tip_theta_joint']
TRAJ_NODE_NAME = 'trajectory_follower_node'
JOY_NODE_NAME = 'joy_teleop_node'
HOMING_NODE_NAME = 'homing_node'
REAL_JOINT_BRIDGE_NODE_NAME = 'real_joint_bridge_node'
# 統合操作タブの「機体ステータス」パネルで起動状況を表示するノード
# (command_gui_nodeがサービス/パラメータ経由で直接やり取りする4つ)。
STATUS_NODE_NAMES = [TRAJ_NODE_NAME, JOY_NODE_NAME, HOMING_NODE_NAME, REAL_JOINT_BRIDGE_NODE_NAME]

# 統合操作タブの「実機セットアップ」パネルから起動する、本番でそのまま使う
# launch構成(note/command.txt「4軸(root_theta/tip_theta/z/r)全軸の実機動作確認。
# 本番でそのまま使う想定」のコマンドと同じ)。real_all_axes_test.launch.pyは
# command_gui_nodeも起動するが、既にこのGUIプロセス自身が動いているため
# launch_gui:=falseでGUIの二重起動を防ぐ。
ALL_AXES_LAUNCH_CMD = [
    'ros2', 'launch', 'soki_sim', 'real_all_axes_test.launch.py',
    'use_joy:=true', 'use_viz:=true', 'launch_gui:=false',
]

# 機体原点オフセット(soki_sim.urdf.xacroのmachine_origin_x/y/z_joint、base_linkの
# 子であるprismaticジョイント)。trajectory_follower_nodeの管理対象外(滑らか追従は
# 不要な較正値)のため、motor_mixer_nodeと同様/mixed_joint_statesへ直接publishする。
MACHINE_ORIGIN_JOINT_NAMES = ['machine_origin_x_joint', 'machine_origin_y_joint', 'machine_origin_z_joint']
# soki_sim.urdf.xacroのmachine_origin_offset_limitと一致させること
MACHINE_ORIGIN_OFFSET_LIMIT = 1.0

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
    ]),
    (None, [
        ('motor1_sign', 'motor1回転方向(±1)', 'float'),
        ('motor2_sign', 'motor2回転方向(±1)', 'float'),
    ]),
]
CUBEMARS_WIRING_FIELDS = [
    (None, [
        ('cubemars_device_id', 'ID', 'int'),
        ('cubemars_root_theta_index', 'root_thetaスロット', 'int'),
        ('cubemars_tip_theta_index', 'tip_thetaスロット', 'int'),
    ]),
    (None, [
        ('root_theta_sign', 'root_theta回転方向(±1)', 'float'),
        ('tip_theta_sign', 'tip_theta回転方向(±1)', 'float'),
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
    ('ワークピッチ変更サーボ', [
        ('pitch_servo_device_id', 'ID', 'int'),
        ('pitch_servo_node_index', 'ノード', 'int'),
        ('pitch_servo_local_index', 'SERVOスロット', 'int'),
    ]),
    (None, [
        ('pitch_servo_hold_deg', '保持角度[deg]', 'int'),
        ('pitch_servo_insert_deg', '投入角度[deg]', 'int'),
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


class CommandGuiNode(Node):

    def __init__(self):
        super().__init__('command_gui_node')
        self.pub_ = self.create_publisher(JointState, 'joint_targets', 10)
        self.mixed_pub_ = self.create_publisher(JointState, 'mixed_joint_states', 10)

        self._current_positions = {name: 0.0 for name in JOINT_NAMES + MACHINE_ORIGIN_JOINT_NAMES}
        self._current_received = False
        self.create_subscription(JointState, 'mixed_joint_states', self._on_mixed_joint_state, 10)

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

    def _on_mixed_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_positions:
                self._current_positions[name] = pos
        self._current_received = True

    def has_current_state(self):
        return self._current_received

    def get_active_node_names(self):
        """現在ROSグラフに存在するノード名の集合を返す(機体ステータスパネルの
        起動状況表示用)。"""
        return set(self.get_node_names())

    def get_current_positions(self):
        return dict(self._current_positions)

    def send_target(self, theta, zj, r):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'auto'
        msg.name = list(JOINT_NAMES)
        msg.position = [theta, zj, r]
        self.pub_.publish(msg)

    def send_machine_origin(self, x, y, z):
        """機体原点オフセット(machine_origin_x/y/z_joint)を/mixed_joint_statesへ
        直接publishする。trajectory_follower_nodeを経由しないため即座に反映される。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MACHINE_ORIGIN_JOINT_NAMES)
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
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                     double_array_value=[float(x) for x in v])
            elif isinstance(v, str):
                pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
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

        self.tabs = QTabWidget()
        root_layout.addWidget(self.tabs)

        manual_tab = QWidget()
        overview_tab = QWidget()
        gain_tab = QWidget()
        calibration_tab = QWidget()
        wiring_tab = QWidget()
        # 統合操作を既定表示にするだけでなく、タブの並びも一番左にする
        # (先に追加した方が左側になる)。
        self.tabs.addTab(overview_tab, '統合操作')
        self.tabs.addTab(gain_tab, 'ゲイン調整')
        self.tabs.addTab(calibration_tab, '原点校正')
        self.tabs.addTab(wiring_tab, '配線設定')
        self.tabs.addTab(manual_tab, '座標指定操作')

        self._build_move_tab(manual_tab)
        self._build_overview_tab(overview_tab)
        self._build_gain_tab(gain_tab)
        self._build_calibration_tab(calibration_tab)
        self._build_wiring_tab(wiring_tab)
        self._restore_saved_gains()

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

    def _build_gain_tab(self, parent):
        self._build_panel_tab(
            parent,
            top_funcs=[self._build_apply_all_panel],
            left_funcs=[self._build_trajectory_panel, self._build_joy_speed_panel],
            right_funcs=[self._build_mit_gain_panel, self._build_robomas_gain_panel])

    def _build_calibration_tab(self, parent):
        self._build_panel_tab(
            parent,
            left_funcs=[self._build_machine_origin_offset_panel, self._build_origin_panel,
                        self._build_homing_panel])

    def _build_wiring_tab(self, parent):
        self._build_panel_tab(
            parent,
            left_funcs=[self._build_robomas_wiring_panel, self._build_cubemars_wiring_panel,
                        self._build_hand_wiring_panel],
            right_funcs=[self._build_homing_wiring_panel, self._build_limit_switch_wiring_panel])

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
            joy = self._collect_joy_speed_values()
        except (ValueError, KeyError):
            QMessageBox.critical(
                self, '入力エラー',
                '軌道生成・MIT・robomas・joy速度のいずれかに数値以外の入力があります')
            return

        reply = QMessageBox.question(
            self, '全ゲイン一括適用の確認',
            'GUIが保持している軌道生成パラメータ・MITゲイン・robomasゲイン・\n'
            'joy速度を、まとめて実機(trajectory_follower_node/joy_teleop_node)\n'
            'へ即座に反映します。\n\n'
            'Kpを大きくするほど保持力・応答性が上がりますが、\n'
            '実機にかかる力も大きくなります。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        self._persist_traj_values(traj)
        self._persist_mit_values(mit)
        self._persist_gains('robomas_gain', robomas)
        self._persist_gains('joy_speed', joy)

        traj_node_values = dict(traj)
        traj_node_values.update(mit)
        traj_node_values.update(robomas)
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

        launch_row = QHBoxLayout()
        launch_btn = QPushButton('全ノード起動')
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
        homing_start_btn = QPushButton('ホーミング開始')
        homing_start_btn.setProperty('variant', 'primary')
        homing_stop_btn = QPushButton('中断')
        homing_skip_btn = QPushButton('スキップ')
        homing_skip_btn.setProperty('variant', 'danger')
        homing_start_btn.clicked.connect(self._on_start_homing)
        homing_stop_btn.clicked.connect(self._on_stop_homing)
        homing_skip_btn.clicked.connect(self._on_skip_homing)
        homing_row.addWidget(homing_start_btn)
        homing_row.addWidget(homing_stop_btn)
        homing_row.addWidget(homing_skip_btn)
        layout.addLayout(homing_row)

        column.addWidget(box)

    def _on_launch_all_nodes(self):
        if self._launch_process is not None and self._launch_process.poll() is None:
            QMessageBox.information(self, '起動済み', '既に起動中です(先に停止してください)')
            return
        reply = QMessageBox.question(
            self, '全ノード起動の確認',
            '{}\n\n'
            '実機を駆動するノード群(real_joint_bridge_node/homing_node/\n'
            'trajectory_follower_node等)を起動します。\n'
            '周囲に人・障害物がないか確認してください。'.format(' '.join(ALL_AXES_LAUNCH_CMD)),
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        try:
            self._launch_process = subprocess.Popen(ALL_AXES_LAUNCH_CMD)
        except OSError as exc:
            _set_status(self.launch_status_label, f'起動失敗: {exc}', 'error')
            return
        _set_status(self.launch_status_label, '起動処理中...', 'muted')

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
        self._launch_process.send_signal(signal.SIGINT)
        _set_status(self.launch_status_label, '停止処理中...', 'muted')

    def _build_current_state_panel(self, column):
        box = QGroupBox('現在状態 (リアルタイム)')
        layout = QVBoxLayout(box)
        self.current_label = QLabel()
        self.current_label.setWordWrap(True)
        _set_status(self.current_label, '(mixed_joint_states待ち)', 'success')
        layout.addWidget(self.current_label)
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
        # ハンド(吸着パッド展開・ワークピッチ変更の2サーボ、ダイヤフラムポンプ)の
        # 操作パネル。配線・角度・デューティの編集は配線設定タブの
        # 「ハンド配線設定」で行い、ここでは4つのサービス(hand_node)を
        # 呼ぶだけにする(定型位置移動ボタンと同様、確認ダイアログは付けない。
        # 展開/収納・ピッチ切替は試合中に繰り返し使う通常操作のため)。
        box = QGroupBox('ハンド (hand_node)')
        layout = QVBoxLayout(box)

        self.hand_status_label = QLabel()
        self.hand_status_label.setWordWrap(True)
        _set_status(self.hand_status_label, '未実行', 'muted')
        layout.addWidget(self.hand_status_label)

        pad_row = QHBoxLayout()
        deploy_btn = QPushButton('吸着パッド展開(吸着ON)')
        deploy_btn.setProperty('variant', 'primary')
        retract_btn = QPushButton('吸着パッド収納(吸着OFF)')
        deploy_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_deploy_pads'))
        retract_btn.clicked.connect(lambda: self._on_hand_trigger('/hand_retract_pads'))
        pad_row.addWidget(deploy_btn)
        pad_row.addWidget(retract_btn)
        layout.addLayout(pad_row)

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
        ok = self.node.call_trigger_service(service_name, self._on_hand_trigger_done)
        if not ok:
            _set_status(self.hand_status_label, 'hand_nodeに接続できません(未起動?)', 'error')

    def _on_hand_trigger_done(self, success, message):
        _set_status(self.hand_status_label, message, 'success' if success else 'error')

    def _build_trajectory_panel(self, column):
        box = QGroupBox('軌道生成パラメータ (trajectory_follower_node)')
        grid = QGridLayout(box)
        grid.addWidget(QLabel('max_vel'), 0, 1)
        grid.addWidget(QLabel('max_accel'), 0, 2)

        self.traj_vel_edits = {}
        self.traj_accel_edits = {}
        for i, name in enumerate(JOINT_NAMES):
            # 行ラベルは"_joint"を省いて表示(ボックス見出しで対象は自明なため、
            # 列幅を無駄に広げないようにする)。辞書キーは元のjoint名のまま。
            grid.addWidget(QLabel(name.removesuffix('_joint')), i + 1, 0)
            vel_edit = make_float_edit(0.0, width=70)
            accel_edit = make_float_edit(0.0, width=70)
            self.traj_vel_edits[name] = vel_edit
            self.traj_accel_edits[name] = accel_edit
            grid.addWidget(vel_edit, i + 1, 1)
            grid.addWidget(accel_edit, i + 1, 2)

        self.traj_status_label = QLabel()
        self.traj_status_label.setWordWrap(True)
        _set_status(self.traj_status_label, '未読込', 'muted')
        grid.addWidget(self.traj_status_label, len(JOINT_NAMES) + 1, 0, 1, 3)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'primary')
        load_btn.clicked.connect(self._on_load_traj_params)
        apply_btn.clicked.connect(self._on_apply_traj_params)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        grid.addLayout(btn_row, len(JOINT_NAMES) + 2, 0, 1, 3)

        column.addWidget(box)

    def _build_joy_speed_panel(self, column):
        box = QGroupBox('手動操作(joy)速度 (joy_teleop_node)')
        grid = QGridLayout(box)
        self.joy_speed_edits = {}
        for i, (name, label, unit) in enumerate((
                ('theta_speed', 'root_theta', 'rad/s'),
                ('z_speed', 'z', 'm/s'),
                ('r_speed', 'r', 'm/s'))):
            grid.addWidget(QLabel(f'{label} [{unit}]'), i, 0)
            edit = make_float_edit(0.0, width=70)
            self.joy_speed_edits[name] = edit
            grid.addWidget(edit, i, 1)

        self.joy_speed_status_label = QLabel()
        self.joy_speed_status_label.setWordWrap(True)
        _set_status(self.joy_speed_status_label, '未読込', 'muted')
        grid.addWidget(self.joy_speed_status_label, 3, 0, 1, 2)

        btn_row = QHBoxLayout()
        load_btn = QPushButton('読込')
        apply_btn = QPushButton('適用')
        apply_btn.setProperty('variant', 'primary')
        load_btn.clicked.connect(self._on_load_joy_speed)
        apply_btn.clicked.connect(self._on_apply_joy_speed)
        btn_row.addWidget(load_btn)
        btn_row.addWidget(apply_btn)
        grid.addLayout(btn_row, 4, 0, 1, 2)

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

    def _build_origin_panel(self, column):
        box = QGroupBox('root_theta原点設定 (CubeMars本体、trajectory_follower_node)')
        layout = QVBoxLayout(box)

        warn = QLabel()
        warn.setWordWrap(True)
        _set_status(warn, '呼び出し前に、root_theta_jointを原点センサの位置\n'
                          '(真の機械原点)へ物理的に合わせておくこと。', 'error')
        layout.addWidget(warn)

        self.origin_status_label = QLabel()
        self.origin_status_label.setWordWrap(True)
        _set_status(self.origin_status_label, '未実行', 'muted')
        layout.addWidget(self.origin_status_label)

        btn = QPushButton('/set_root_theta_origin 呼び出し')
        btn.setProperty('variant', 'danger')
        btn.clicked.connect(self._on_set_root_theta_origin)
        layout.addWidget(btn)

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
                 '内蔵ロータエンコーダのCAN帰還を位置の真値として使う。',
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
            yaml_filename='hand.yaml')

    def _build_yaml_wiring_panel(self, column, attr_prefix, title, description, field_specs, note=None,
                                  yaml_filename='real_joint_bridge.yaml'):
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

    def _on_load_yaml_wiring(self, attr_prefix, field_specs, yaml_filename='real_joint_bridge.yaml'):
        edits = getattr(self, f'_{attr_prefix}_edits')
        status_label = getattr(self, f'_{attr_prefix}_status_label')
        path = _resolve_config_yaml_path(yaml_filename)
        if path is None:
            _set_status(status_label, f'{yaml_filename}が見つかりません', 'error')
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as exc:
            _set_status(status_label, f'読込失敗: {exc}', 'error')
            return
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
        box = QGroupBox('ワーク・シューティングボックス (クリックで移動、実フィールド配置)')
        grid = QGridLayout(box)
        work_points = [p for row in WORK_POINTS for p in row]
        shoot_points = SHOOT_POINTS['L'] + SHOOT_POINTS['R']
        n_work_rows = len({round(p[2], 6) for p in work_points})
        self._build_button_grid(grid, work_points + shoot_points, header_row=1,
                                 gap_after_rank=n_work_rows - 1, gap_label='(機体)')
        # QVBoxLayoutへ直接addWidgetすると幅いっぱいに引き伸ばされてしまう
        # (グリッド内容は中身ぴったりのまま、右側だけ間延びした余白ができる)ため、
        # 横方向はQHBoxLayout+末尾stretchで中身ぴったりの幅に留める。
        row = QHBoxLayout()
        row.addWidget(box)
        row.addStretch(1)
        layout.addLayout(row)

    def _build_button_grid(self, grid, points, header_row=0, gap_after_rank=None, gap_label=''):
        """points: (label, x, y, z)のリスト。実座標を見た目通りに配置する
        (X昇順=左->右の列、Y降順=奥(ワーク方向)が上->手前が下の行)。
        gap_after_rankを指定すると、Y順位でその順位を超えた行を1行分下にずらし、
        間にgap_labelを挟む(ワーク行とシューティングボックス行の間に機体分の
        空白を作るため)。"""
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
        btn_width = max(metrics.horizontalAdvance(label) for label, _, _, _ in points) + 14
        for label, x, y, z in points:
            col = xs.index(round(x, 6))
            rank = ys.index(round(y, 6))
            if gap_after_rank is not None and rank > gap_after_rank:
                rank += 1
            btn = QPushButton(label)
            btn.setProperty('variant', 'grid')
            btn.setFixedWidth(btn_width)
            btn.clicked.connect(lambda _checked=False, x=x, y=y, z=z: self._on_field_point(x, y, z))
            grid.addWidget(btn, rank + header_row, col)

        has_gap = gap_after_rank is not None
        if has_gap and gap_label:
            gap_widget = QLabel(gap_label)
            gap_widget.setAlignment(Qt.AlignCenter)
            _set_status(gap_widget, gap_label, 'muted')
            grid.addWidget(gap_widget, header_row + gap_after_rank + 1, 0, 1, ncols)

        total_rows = len(ys) + (1 if has_gap else 0)
        grid.addWidget(QLabel('↑ Y+ (ワーク側) ／ Y- (機体側) ↓'), header_row + total_rows, 0, 1, ncols)

    def _on_field_point(self, x, y, z):
        self._send_xyz(x, y, z)

    # ---------- realtime state / trajectory params ----------
    def _spin_ros(self):
        # rclpy.spin_once()はmixed_joint_states購読・パラメータサービスの
        # 応答処理に必要(このメソッドの呼び出し=QTimer=Qtのイベントループと
        # 同じメインスレッド上で完結する)。
        rclpy.spin_once(self.node, timeout_sec=0)
        self._refresh_current_state()

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

    def _on_load_traj_params(self):
        # joint_namesも取得する: trajectory_follower_nodeは実行構成によって
        # JOINT_NAMES(root_theta/z/r)の全部ではなく一部だけで起動されることがある
        # (例: real_root_theta_test.launch.pyはroot_theta_jointのみ)。max_velocity等の
        # 配列は「そのノードの実際のjoint_names順」なので、GUI固定のJOINT_NAMESを
        # 前提にlen比較・zipすると、一致しない構成では表示が更新されず0のままに見える。
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME, ['max_velocity', 'max_acceleration', 'control_mode', 'joint_names'],
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
        if not self._joy_auto_applied and self._auto_apply_saved_joy():
            self._joy_auto_applied = True

        if all((self._traj_auto_loaded, self._mit_auto_loaded,
                self._robomas_auto_loaded, self._joy_auto_loaded,
                self._traj_auto_applied, self._mit_auto_applied,
                self._robomas_auto_applied, self._joy_auto_applied)):
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
        if not all(n in vel_map for n in names) or not all(n in accel_map for n in names):
            return False
        payload = {
            'max_velocity': [vel_map[n] for n in names],
            'max_acceleration': [accel_map[n] for n in names],
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

    def _auto_apply_saved_joy(self):
        saved = self._saved_gains.get('joy_speed')
        if not saved:
            return True
        _set_status(self.joy_speed_status_label, '自動適用中(gains.json)...', 'muted')
        return self.node.set_node_params(JOY_NODE_NAME, saved, self._apply_joy_speed_set_result)

    def _apply_loaded_traj_params(self, values):
        vel = values.get('max_velocity')
        accel = values.get('max_acceleration')
        mode = values.get('control_mode')
        names = values.get('joint_names') or JOINT_NAMES
        self._traj_joint_names = list(names)
        # request_node_paramsは非同期(応答はrclpy.spin_once経由でこのコールバックが
        # 呼ばれて初めて届く)。_try_auto_setup_gainsはこのフラグを見てから
        # _auto_apply_saved_trajを呼ぶことで、_traj_joint_namesが実際の構成に
        # 更新される前(読込リクエストを送っただけの段階)に古い関節数のまま
        # 適用してしまい「4要素必要」等で失敗する問題を防ぐ(2026-09-02修正)。
        self._traj_names_known = True
        missing = [n for n in JOINT_NAMES if n not in names]
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
        「読込」未実行でもGUI固定のJOINT_NAMESを対象に動作する(__init__参照)。
        tip_theta_joint等、GUIに編集欄が無い関節(JOINT_NAMES対象外)がノード側の
        joint_namesに含まれる場合は、_traj_loaded_extraに保持した読込時の値を
        そのまま使い、その関節の設定値を変更せず送り直す(real_all_axes_test.
        launch.py参照)。"""
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
        return {'max_velocity': vel, 'max_acceleration': accel}

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
            TRAJ_NODE_NAME, ['robomas_kp', 'robomas_kd', 'robomas_current_ff', 'robomas_device_id'],
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
        }

    def _on_apply_robomas_gains(self):
        try:
            values = self._collect_robomas_values()
        except ValueError:
            QMessageBox.critical(self, '入力エラー', 'Kp/Kd/current_ffに数値を入力してください')
            return
        reply = QMessageBox.question(
            self, 'MITゲイン適用の確認',
            'motor1/motor2(z/r)のMITゲインを実機へ即座に反映します。\n'
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

    def _build_homing_panel(self, column):
        box = QGroupBox('z/rホーミング (homing_node)')
        layout = QVBoxLayout(box)

        desc = QLabel()
        desc.setWordWrap(True)
        _set_status(desc, '開始: motor1/motor2を低速駆動し原点センサまで動かす。\n'
                          'スキップ: 機体を先に原点センサ位置相当へ手動で\n'
                          '合わせてから使うこと(モータは駆動しない)。', 'muted')
        layout.addWidget(desc)

        self.homing_status_label = QLabel()
        self.homing_status_label.setWordWrap(True)
        _set_status(self.homing_status_label, '未実行', 'muted')
        layout.addWidget(self.homing_status_label)

        btn_row = QHBoxLayout()
        start_btn = QPushButton('開始')
        start_btn.setProperty('variant', 'primary')
        stop_btn = QPushButton('中断')
        skip_btn = QPushButton('スキップ')
        skip_btn.setProperty('variant', 'danger')
        start_btn.clicked.connect(self._on_start_homing)
        stop_btn.clicked.connect(self._on_stop_homing)
        skip_btn.clicked.connect(self._on_skip_homing)
        btn_row.addWidget(start_btn)
        btn_row.addWidget(stop_btn)
        btn_row.addWidget(skip_btn)
        layout.addLayout(btn_row)

        column.addWidget(box)

    def _on_start_homing(self):
        reply = QMessageBox.question(
            self, 'ホーミング開始の確認',
            'motor1/motor2を低速駆動してz/r原点センサまで動かします。\n'
            '周囲に人・障害物がないか確認してください。',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        _set_status(self.homing_status_label, '開始中...', 'muted')
        ok = self.node.call_trigger_service('/start_homing', self._on_homing_service_done)
        if not ok:
            _set_status(self.homing_status_label, 'サービス未起動です(homing_node起動確認)', 'error')

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
            JOY_NODE_NAME, ['theta_speed', 'z_speed', 'r_speed'],
            self._apply_loaded_joy_speed,
            lambda reason: _set_status(self.joy_speed_status_label, f'読込失敗: {reason}', 'error'))
        _set_status(self.joy_speed_status_label,
                    '読込中...' if ok else 'joy_teleop_nodeに接続できません(use_joy:=trueで起動?)',
                    'muted' if ok else 'error')
        return ok

    def _apply_loaded_joy_speed(self, values):
        for name in ('theta_speed', 'z_speed', 'r_speed'):
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
        for key, edits in (('max_velocity', self.traj_vel_edits), ('max_acceleration', self.traj_accel_edits)):
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

        joy = self._saved_gains.get('joy_speed', {})
        for name, edit in self.joy_speed_edits.items():
            if name in joy:
                set_float(edit, joy[name])

    def closeEvent(self, event):
        """実機セットアップパネルで起動したros2 launch子プロセスが残っている場合、
        GUI終了時に道連れで放置されないよう確認して停止する(通常のCtrl+Cと同様の
        SIGINTで、ros2 launch側に配下ノードをまとめて終了させる)。"""
        if self._launch_process is not None and self._launch_process.poll() is None:
            reply = QMessageBox.question(
                self, '終了確認',
                '実機セットアップで起動したノード群がまだ動作中です。\n'
                '停止してから終了しますか？',
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            if reply == QMessageBox.Cancel:
                event.ignore()
                return
            if reply == QMessageBox.Yes:
                self._launch_process.send_signal(signal.SIGINT)
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
