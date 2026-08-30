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
import sys

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QColor, QDoubleValidator, QIcon, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QGridLayout, QGroupBox, QHBoxLayout,
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

# 機体原点オフセット(soki_sim.urdf.xacroのmachine_origin_x/y/z_joint、base_linkの
# 子であるprismaticジョイント)。trajectory_follower_nodeの管理対象外(滑らか追従は
# 不要な較正値)のため、motor_mixer_nodeと同様/mixed_joint_statesへ直接publishする。
MACHINE_ORIGIN_JOINT_NAMES = ['machine_origin_x_joint', 'machine_origin_y_joint', 'machine_origin_z_joint']
# soki_sim.urdf.xacroのmachine_origin_offset_limitと一致させること
MACHINE_ORIGIN_OFFSET_LIMIT = 0.1


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
    'info': '#5aa9e6',
    'success': '#57c278',
    'error': '#f2545b',
    'muted': '#9aa0a6',
}


def _set_status(label: QLabel, text: str, role: str = 'muted'):
    """ステータス表示用QLabelのテキストと色をまとめて設定する(Tkinter版のfg=動的
    変更に相当)。"""
    label.setText(text)
    label.setStyleSheet(f'color: {_ROLE_COLORS[role]};')


def make_float_edit(initial: float, width: int = 80) -> QLineEdit:
    """数値入力用QLineEdit(Tkinter版のtk.Entry+DoubleVarに相当)を生成する。"""
    edit = QLineEdit()
    edit.setValidator(QDoubleValidator(-1.0e6, 1.0e6, 6))
    edit.setMaximumWidth(width)
    set_float(edit, initial)
    return edit


def get_float(edit: QLineEdit) -> float:
    """QLineEditの内容をfloatとして取得する。数値でない場合はValueErrorを送出する
    (Tkinter版のDoubleVar.get()がtk.TclErrorを送出するのに相当)。"""
    text = edit.text().strip()
    if text in ('', '-', '.', '-.'):
        raise ValueError(text)
    return float(text)


def set_float(edit: QLineEdit, value: float):
    edit.setText(f'{value:.6g}')


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
        self._traj_joint_names = list(JOINT_NAMES)
        self._mit_joint_names = []
        self._mode_buttons = {}

        self.x_edit = make_float_edit(MAX_RADIUS / 2.0)
        self.y_edit = make_float_edit(0.0)
        self.z_edit = make_float_edit((WORLD_Z_LOWER + WORLD_Z_UPPER) / 2.0)
        self.step_edit = make_float_edit(0.01, width=60)

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(8, 8, 8, 8)

        # パラメータパネル(統合操作タブ)が増えるにつれ縦に伸び、ワーク/シューティング
        # ボックスのボタン(元はマニュアル操作タブ側)が画面外に押し出される問題が
        # 起きたため、QTabWidgetで「マニュアル操作」(円クリック・ジョグ・保存済み
        # ポイント等、自由な位置への移動系)と「統合操作」(各種パラメータパネル、
        # およびワーク/シューティングボックスの定型位置移動ボタン)を別タブに分離
        # している。起動時に表示するタブは、動作モード切替や各種パラメータなど
        # 主要な操作をまとめた統合操作タブをデフォルトにする。
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
        integrated_tab = QWidget()
        self.tabs.addTab(manual_tab, 'マニュアル操作')
        self.tabs.addTab(integrated_tab, '統合操作')

        self._build_move_tab(manual_tab)
        self._build_settings_tab(integrated_tab)

        self.tabs.setCurrentWidget(integrated_tab)

        for edit in (self.x_edit, self.y_edit, self.z_edit):
            edit.textChanged.connect(self._redraw_pin)
        self._redraw_pin()
        self._refresh_point_list()

        self.setMinimumWidth(560)
        self.resize(760, 820)

        # rclpy.spin_once()はmixed_joint_states購読・パラメータサービスの応答処理に
        # 必要(このタイマーのコールバック=Qtのイベントループと同じメインスレッド上で
        # 完結する。別スレッドのexecutorにしない理由はモジュールdocstring参照)。
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros)
        self._spin_timer.start(50)

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

    # ---------- settings (integrated) tab ----------
    def _build_settings_tab(self, parent):
        outer = QVBoxLayout(parent)
        outer.setContentsMargins(0, 0, 0, 0)

        # パネルが増えても画面からはみ出さないよう、スクロール可能な領域に入れる。
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        inner_layout = QVBoxLayout(inner)

        self._build_field_buttons(inner_layout)

        columns = QHBoxLayout()
        left_col = QVBoxLayout()
        right_col = QVBoxLayout()
        columns.addLayout(left_col)
        columns.addLayout(right_col)
        inner_layout.addLayout(columns)
        inner_layout.addStretch(1)

        self._build_current_state_panel(left_col)
        self._build_mode_panel(left_col)
        self._build_machine_origin_offset_panel(left_col)
        self._build_origin_panel(left_col)

        self._build_trajectory_panel(right_col)
        self._build_joy_speed_panel(right_col)
        self._build_mit_gain_panel(right_col)
        self._build_robomas_gain_panel(right_col)
        self._build_homing_panel(right_col)

        scroll.setWidget(inner)
        outer.addWidget(scroll)

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

    def _build_trajectory_panel(self, column):
        box = QGroupBox('軌道生成パラメータ (trajectory_follower_node)')
        grid = QGridLayout(box)
        grid.addWidget(QLabel('max_vel'), 0, 1)
        grid.addWidget(QLabel('max_accel'), 0, 2)

        self.traj_vel_edits = {}
        self.traj_accel_edits = {}
        for i, name in enumerate(JOINT_NAMES):
            grid.addWidget(QLabel(name), i + 1, 0)
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
            grid.addWidget(QLabel(name), i + 1, 0)
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

    def _build_field_buttons(self, layout):
        field_row = QHBoxLayout()

        work_box = QGroupBox('ワーク (クリックで移動)')
        work_grid = QGridLayout(work_box)
        self._build_button_grid(work_grid, [p for row in WORK_POINTS for p in row], header_row=1)
        field_row.addWidget(work_box)

        shoot_box = QGroupBox('シューティングボックス (クリックで移動)')
        shoot_grid = QGridLayout(shoot_box)
        self._build_button_grid(shoot_grid, SHOOT_POINTS['L'] + SHOOT_POINTS['R'], header_row=1)
        field_row.addWidget(shoot_box)

        field_row.addStretch(1)
        layout.addLayout(field_row)

    def _build_button_grid(self, grid, points, header_row=0):
        """points: (label, x, y, z)のリスト。実座標を見た目通りに配置する
        (X昇順=左->右の列、Y降順=奥(ワーク方向)が上->手前が下の行)。"""
        grid.addWidget(QLabel('← X- ・ X+ →'), 0, 0, 1, 99)
        xs = sorted({round(p[1], 6) for p in points})
        ys = sorted({round(p[2], 6) for p in points}, reverse=True)
        for label, x, y, z in points:
            col = xs.index(round(x, 6))
            row = ys.index(round(y, 6)) + header_row
            btn = QPushButton(label)
            btn.setFixedWidth(48)
            btn.clicked.connect(lambda _checked=False, x=x, y=y, z=z: self._on_field_point(x, y, z))
            grid.addWidget(btn, row, col)
        grid.addWidget(QLabel('↑ Y+ (ワーク側) ／ Y- (機体側) ↓'), header_row + len(ys), 0, 1, 99)

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

    def _apply_loaded_traj_params(self, values):
        vel = values.get('max_velocity')
        accel = values.get('max_acceleration')
        mode = values.get('control_mode')
        names = values.get('joint_names') or JOINT_NAMES
        self._traj_joint_names = list(names)
        missing = [n for n in JOINT_NAMES if n not in names]
        if vel and len(vel) == len(names):
            for name, v in zip(names, vel):
                if name in self.traj_vel_edits:
                    set_float(self.traj_vel_edits[name], round(v, 4))
        if accel and len(accel) == len(names):
            for name, v in zip(names, accel):
                if name in self.traj_accel_edits:
                    set_float(self.traj_accel_edits[name], round(v, 4))
        if mode:
            self._set_mode_silent(mode)
            _set_status(self.mode_status_label, f'現在のモード: {mode}', 'info')
        status = '読込完了'
        if missing:
            status += f' (未起動構成: {", ".join(missing)}は表示更新されません)'
        _set_status(self.traj_status_label, status, 'info')

    def _on_apply_traj_params(self):
        # 実際にtrajectory_follower_nodeが持つjoint_names順で配列を組む
        # (GUI固定のJOINT_NAMESで組むと、一部関節のみの構成では長さ不一致で
        # set_parametersに拒否される)。
        names = self._traj_joint_names
        try:
            vel = [get_float(self.traj_vel_edits[name]) for name in names]
            accel = [get_float(self.traj_accel_edits[name]) for name in names]
        except (ValueError, KeyError):
            QMessageBox.critical(self, '入力エラー', '速度・加速度に数値を入力してください')
            return
        ok = self.node.set_node_params(
            TRAJ_NODE_NAME, {'max_velocity': vel, 'max_acceleration': accel},
            self._apply_traj_set_result)
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

    def _apply_loaded_mit_gains(self, values):
        kp = values.get('cubemars_kp')
        kd = values.get('cubemars_kd')
        tff = values.get('cubemars_torque_ff')
        names = values.get('cubemars_joint_names') or []
        self._mit_joint_names = list(names)
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
        status = '読込完了'
        not_configured = [n for n in CUBEMARS_JOINT_NAMES if n not in names]
        if not_configured:
            status += f' (実機出力対象外: {", ".join(not_configured)})'
        if not names:
            status = '読込完了 (実機出力対象の関節が設定されていません)'
        _set_status(self.mit_gain_status_label, status, 'info')

    def _on_apply_mit_gains(self):
        # cubemars_joint_namesの実際の順序(_on_load_mit_gainsで取得済みのもの)で
        # 配列を組む必要がある。未読込のまま適用すると対象関節・順序が不明なため、
        # 先に読込を要求する(誤った関節にゲインを適用する事故を防ぐ)。
        names = self._mit_joint_names
        if not names:
            QMessageBox.critical(self, '未読込', '先に「読込」を実行して対象関節を確認してください')
            return
        try:
            kp = [get_float(self.mit_kp_edits[name]) for name in names]
            kd = [get_float(self.mit_kd_edits[name]) for name in names]
            tff = [get_float(self.mit_torque_edits[name]) for name in names]
        except (ValueError, KeyError):
            QMessageBox.critical(self, '入力エラー', 'Kp/Kd/torque_ffに数値を入力してください')
            return
        reply = QMessageBox.question(
            self, 'MITゲイン適用の確認',
            f'{", ".join(names)} のMITゲインを実機へ即座に反映します。\n'
            'Kpを大きくするほど保持力・応答性が上がりますが、\n'
            '実機にかかる力も大きくなります。よろしいですか？',
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        ok = self.node.set_node_params(
            TRAJ_NODE_NAME,
            {'cubemars_kp': kp, 'cubemars_kd': kd, 'cubemars_torque_ff': tff},
            self._apply_mit_gain_set_result)
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

    def _apply_loaded_robomas_gains(self, values):
        if 'robomas_kp' in values:
            set_float(self.robomas_kp_edit, round(values['robomas_kp'], 6))
        if 'robomas_kd' in values:
            set_float(self.robomas_kd_edit, round(values['robomas_kd'], 6))
        if 'robomas_current_ff' in values:
            set_float(self.robomas_current_ff_edit, round(values['robomas_current_ff'], 6))
        device_id = values.get('robomas_device_id', 0)
        status = '読込完了'
        if not device_id:
            status += ' (実機出力無効: robomas_device_id未設定)'
        else:
            status += f' (device_id={device_id})'
        _set_status(self.robomas_gain_status_label, status, 'info')

    def _on_apply_robomas_gains(self):
        try:
            values = {
                'robomas_kp': get_float(self.robomas_kp_edit),
                'robomas_kd': get_float(self.robomas_kd_edit),
                'robomas_current_ff': get_float(self.robomas_current_ff_edit),
            }
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

    def _apply_loaded_joy_speed(self, values):
        for name in ('theta_speed', 'z_speed', 'r_speed'):
            if name in values:
                set_float(self.joy_speed_edits[name], round(values[name], 4))
        _set_status(self.joy_speed_status_label, '読込完了', 'info')

    def _on_apply_joy_speed(self):
        try:
            values = {name: get_float(edit) for name, edit in self.joy_speed_edits.items()}
        except ValueError:
            QMessageBox.critical(self, '入力エラー', '速度に数値を入力してください')
            return
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
