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
購読・サービス呼び出しの処理にはspinが必要なため、Tkinterのafter()による定期
ポーリングループの中でrclpy.spin_once(timeout_sec=0)を呼び出す(mainloopと
同じメインスレッドで完結させるシングルスレッド構成。executorを別スレッドで
回す構成も試したが、このROS 2 Jazzy環境ではサービスクライアントを使った
場合にプロセス終了時のrmw層クリーンアップがスレッド競合でAborted(core dumped)
になることを確認したため採用しなかった)。

tkinterが必要(未インストールの場合: sudo apt install python3-tk)。
"""
import json
import math
import os
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

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
        呼び出し中(Tkinterのafter()ループと同じメインスレッド)に呼ばれる。"""
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
        呼び出し中(Tkinterのafter()ループと同じメインスレッド)に呼ばれる。"""
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
        (Tkinterのafter()ループと同じメインスレッド)に呼ばれる。サービス未起動の
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


class CommandGuiApp(tk.Tk):
    CANVAS_SIZE = 440
    MARGIN = 20

    def __init__(self, node: CommandGuiNode):
        super().__init__()
        self.node = node
        self.title('soki_sim command GUI')

        self.points = self._load_points()

        self.x_var = tk.DoubleVar(value=MAX_RADIUS / 2.0)
        self.y_var = tk.DoubleVar(value=0.0)
        self.z_var = tk.DoubleVar(value=(WORLD_Z_LOWER + WORLD_Z_UPPER) / 2.0)
        self.step_var = tk.DoubleVar(value=0.01)

        self._build_widgets()
        # trace_addは_build_widgets()より後で登録すること: 先に登録すると、
        # coord_frameのEntry(textvariable=x_var等)を生成した時点でTkinterが
        # 変数の書き込みイベントを発生させ、_redraw_pin->_update_statusが
        # self.status_label作成(_build_widgets()の終盤)より前に呼ばれてしまう。
        for var in (self.x_var, self.y_var, self.z_var):
            var.trace_add('write', lambda *_: self._redraw_pin())
        self._redraw_pin()
        self._refresh_point_list()
        self.after(50, self._spin_ros)

    # ---------- widgets ----------
    def _build_widgets(self):
        # パラメータパネル(統合操作タブ)が増えるにつれ縦に伸び、ワーク/シューティング
        # ボックスのボタン(元はマニュアル操作タブ側)が画面外に押し出される問題が
        # 起きたため、ttk.Notebookで「マニュアル操作」(円クリック・ジョグ・保存済み
        # ポイント等、自由な位置への移動系)と「統合操作」(各種パラメータパネル、
        # およびワーク/シューティングボックスの定型位置移動ボタン)を別タブに分離した
        # (2026-08-27)。起動時に表示するタブは、動作モード切替や各種パラメータなど
        # 主要な操作をまとめた統合操作タブをデフォルトにする(2026-08-29)。
        notebook = ttk.Notebook(self)
        notebook.pack(fill=tk.BOTH, expand=True)

        manual_tab = tk.Frame(notebook)
        integrated_tab = tk.Frame(notebook)
        notebook.add(manual_tab, text='マニュアル操作')
        notebook.add(integrated_tab, text='統合操作')

        self._build_move_tab(manual_tab)
        self._build_settings_tab(integrated_tab)

        notebook.select(integrated_tab)

    def _build_move_tab(self, parent):
        main = tk.Frame(parent, padx=8, pady=8)
        main.pack(fill=tk.BOTH, expand=True)

        left = tk.Frame(main)
        left.pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(left, text='XY平面 (クリックでピン設置。上=ワーク側(Y+)、右=X+)').pack()
        self.canvas = tk.Canvas(left, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE,
                                 background='white', highlightthickness=1,
                                 highlightbackground='grey')
        self.canvas.pack()
        self.canvas.bind('<Button-1>', self._on_canvas_click)
        self._draw_canvas_base()

        mid = tk.Frame(main)
        mid.pack(side=tk.LEFT, padx=8, fill=tk.Y)

        coord_frame = tk.LabelFrame(mid, text='目標座標 [m]')
        coord_frame.pack(fill=tk.X, pady=(0, 8))
        for i, (label, var) in enumerate((('X', self.x_var), ('Y', self.y_var), ('Z', self.z_var))):
            tk.Label(coord_frame, text=label).grid(row=i, column=0, sticky='w')
            tk.Entry(coord_frame, textvariable=var, width=10).grid(row=i, column=1, padx=4, pady=2)
        tk.Button(coord_frame, text='現在位置を目標にコピー', command=self._on_copy_current_to_target
                  ).grid(row=3, column=0, columnspan=2, pady=(4, 2), sticky='we')

        jog_frame = tk.LabelFrame(mid, text='ジョグ (↑ワーク側 ↓機体側 ←X- →X+)')
        jog_frame.pack(fill=tk.X, pady=(0, 8))
        step_row = tk.Frame(jog_frame)
        step_row.pack(pady=(2, 4))
        tk.Label(step_row, text='ステップ[m]').pack(side=tk.LEFT)
        tk.Entry(step_row, textvariable=self.step_var, width=6).pack(side=tk.LEFT, padx=4)

        pad = tk.Frame(jog_frame)
        pad.pack(pady=(0, 4))
        tk.Button(pad, text='↑', width=3, command=lambda: self._on_jog(0, 1)).grid(row=0, column=1)
        tk.Button(pad, text='←', width=3, command=lambda: self._on_jog(-1, 0)).grid(row=1, column=0)
        tk.Button(pad, text='→', width=3, command=lambda: self._on_jog(1, 0)).grid(row=1, column=2)
        tk.Button(pad, text='↓', width=3, command=lambda: self._on_jog(0, -1)).grid(row=2, column=1)

        tk.Label(mid, text=f'Z [{WORLD_Z_LOWER:.2f} - {WORLD_Z_UPPER:.2f} m]').pack()
        tk.Scale(mid, variable=self.z_var, from_=WORLD_Z_UPPER, to=WORLD_Z_LOWER,
                 resolution=0.005, orient=tk.VERTICAL, length=200).pack()

        self.status_label = tk.Label(mid, text='', fg='blue', justify=tk.LEFT)
        self.status_label.pack(pady=4)

        tk.Button(mid, text='送信 (Send)', command=self._on_send,
                  bg='#4a90d9', fg='white').pack(fill=tk.X, pady=(4, 0))

        right = tk.Frame(main)
        right.pack(side=tk.LEFT, padx=(8, 0), fill=tk.BOTH, expand=True)
        tk.Label(right, text='保存済みポイント (ダブルクリックで読込)').pack()
        self.listbox = tk.Listbox(right, width=30, height=14)
        self.listbox.pack(fill=tk.BOTH, expand=True)
        self.listbox.bind('<Double-Button-1>', lambda e: self._on_load_point())

        btns = tk.Frame(right)
        btns.pack(fill=tk.X, pady=4)
        tk.Button(btns, text='追加', command=self._on_add_point).pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(btns, text='送信', command=self._on_send_selected).pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(btns, text='削除', command=self._on_delete_point).pack(side=tk.LEFT, expand=True, fill=tk.X)

    def _build_settings_tab(self, parent):
        # パネルが縦に長くなっても画面からはみ出さないよう、スクロール可能な
        # 領域に入れる(2026-08-27、ワーク/シューティングボックスが画面外に
        # 押し出された問題を受けて、今後パネルが増えても同じ罠を踏まないため)。
        # ワーク/シューティングボックス(定型位置への移動)は「マニュアル操作」タブの
        # 自由位置移動系(円クリック・ジョグ・保存済みポイント)とは性質が違う
        # ("統合操作"に近い定型操作)ため、こちらへ移した(2026-08-27)。
        settings_col = self._make_scrollable(parent)
        self._build_field_buttons(settings_col)  # 2つのLabelFrameが横並びで幅を取るため全幅のまま

        # 縦一列だと画面をはみ出しやすいため、残りのパネルは2カラムに分けて
        # 横に並べる(2026-08-27、スクロールバーだけでは気づかれにくいとの
        # フィードバックを受けて、そもそもの縦の長さを減らす対応も追加)。
        columns = tk.Frame(settings_col)
        columns.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        left_col = tk.Frame(columns)
        left_col.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8), anchor='n')
        right_col = tk.Frame(columns)
        right_col.pack(side=tk.LEFT, fill=tk.Y, anchor='n')

        self._build_current_state_panel(left_col)
        self._build_mode_panel(left_col)
        self._build_machine_origin_offset_panel(left_col)
        self._build_origin_panel(left_col)

        self._build_trajectory_panel(right_col)
        self._build_joy_speed_panel(right_col)
        self._build_mit_gain_panel(right_col)

    def _make_scrollable(self, parent):
        """parent一杯に縦スクロール可能な内側Frameを作って返す。"""
        canvas = tk.Canvas(parent, highlightthickness=0)
        scrollbar = tk.Scrollbar(parent, orient=tk.VERTICAL, command=canvas.yview)
        inner = tk.Frame(canvas, padx=8, pady=8)
        inner.bind('<Configure>', lambda _e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # マウスホイールでもスクロールできるようにする(スクロールバーのドラッグ
        # だけでは気づかれにくく「スクロールできない」ように見える、2026-08-27)。
        # カーソルがこの領域上にある間だけbind_allする(離れたら解除し、他の
        # ウィジェットのホイール動作を奪わない)。Linux(X11/XWayland)は
        # <Button-4/5>、Windows/macOSは<MouseWheel>(event.deltaの符号で判定)。
        def _on_wheel(event):
            if event.num == 4 or getattr(event, 'delta', 0) > 0:
                canvas.yview_scroll(-1, 'units')
            elif event.num == 5 or getattr(event, 'delta', 0) < 0:
                canvas.yview_scroll(1, 'units')

        def _bind_wheel(_e):
            canvas.bind_all('<Button-4>', _on_wheel)
            canvas.bind_all('<Button-5>', _on_wheel)
            canvas.bind_all('<MouseWheel>', _on_wheel)

        def _unbind_wheel(_e):
            canvas.unbind_all('<Button-4>')
            canvas.unbind_all('<Button-5>')
            canvas.unbind_all('<MouseWheel>')

        canvas.bind('<Enter>', _bind_wheel)
        canvas.bind('<Leave>', _unbind_wheel)

        return inner

    def _build_current_state_panel(self, parent):
        frame = tk.LabelFrame(parent, text='現在状態 (リアルタイム)')
        frame.pack(fill=tk.X)
        self.current_label = tk.Label(frame, text='(mixed_joint_states待ち)',
                                       fg='#1a7a1a', justify=tk.LEFT)
        self.current_label.pack(padx=4, pady=4, anchor='w')

    def _build_mode_panel(self, parent):
        frame = tk.LabelFrame(parent, text='動作モード (trajectory_follower_node)')
        frame.pack(fill=tk.X, pady=(8, 0))

        self.mode_var = tk.StringVar(value='both')
        for value, label in (('auto', '自動専用 (GUI)'), ('manual', '手動専用 (joy)'), ('both', '併用')):
            tk.Radiobutton(frame, text=label, variable=self.mode_var, value=value,
                           command=self._on_mode_changed).pack(anchor='w', padx=4)

        self.mode_status_label = tk.Label(frame, text='未読込', fg='grey',
                                           wraplength=220, justify=tk.LEFT)
        self.mode_status_label.pack(padx=4, pady=(2, 4), anchor='w')

    def _build_trajectory_panel(self, parent):
        frame = tk.LabelFrame(parent, text='軌道生成パラメータ (trajectory_follower_node)')
        frame.pack(fill=tk.X, pady=(8, 0))

        tk.Label(frame, text='max_vel').grid(row=0, column=1)
        tk.Label(frame, text='max_accel').grid(row=0, column=2)

        self.traj_vel_vars = {}
        self.traj_accel_vars = {}
        for i, name in enumerate(JOINT_NAMES):
            tk.Label(frame, text=name).grid(row=i + 1, column=0, sticky='w')
            vel_var = tk.DoubleVar(value=0.0)
            accel_var = tk.DoubleVar(value=0.0)
            self.traj_vel_vars[name] = vel_var
            self.traj_accel_vars[name] = accel_var
            tk.Entry(frame, textvariable=vel_var, width=8).grid(row=i + 1, column=1, padx=2, pady=2)
            tk.Entry(frame, textvariable=accel_var, width=8).grid(row=i + 1, column=2, padx=2, pady=2)

        self.traj_status_label = tk.Label(frame, text='未読込', fg='grey',
                                           wraplength=220, justify=tk.LEFT)
        self.traj_status_label.grid(row=len(JOINT_NAMES) + 1, column=0, columnspan=3, pady=(4, 0))

        btn_row = tk.Frame(frame)
        btn_row.grid(row=len(JOINT_NAMES) + 2, column=0, columnspan=3, pady=4, sticky='we')
        tk.Button(btn_row, text='読込', command=self._on_load_traj_params).pack(
            side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(btn_row, text='適用', command=self._on_apply_traj_params,
                  bg='#4a90d9', fg='white').pack(side=tk.LEFT, expand=True, fill=tk.X)

    def _build_joy_speed_panel(self, parent):
        frame = tk.LabelFrame(parent, text='手動操作(joy)速度 (joy_teleop_node)')
        frame.pack(fill=tk.X, pady=(8, 0))

        self.joy_speed_vars = {}
        for i, (name, label, unit) in enumerate((
                ('theta_speed', 'root_theta', 'rad/s'),
                ('z_speed', 'z', 'm/s'),
                ('r_speed', 'r', 'm/s'))):
            tk.Label(frame, text=f'{label} [{unit}]').grid(row=i, column=0, sticky='w')
            var = tk.DoubleVar(value=0.0)
            self.joy_speed_vars[name] = var
            tk.Entry(frame, textvariable=var, width=8).grid(row=i, column=1, padx=2, pady=2)

        self.joy_speed_status_label = tk.Label(frame, text='未読込', fg='grey',
                                                wraplength=220, justify=tk.LEFT)
        self.joy_speed_status_label.grid(row=3, column=0, columnspan=2, pady=(4, 0))

        btn_row = tk.Frame(frame)
        btn_row.grid(row=4, column=0, columnspan=2, pady=4, sticky='we')
        tk.Button(btn_row, text='読込', command=self._on_load_joy_speed).pack(
            side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(btn_row, text='適用', command=self._on_apply_joy_speed,
                  bg='#4a90d9', fg='white').pack(side=tk.LEFT, expand=True, fill=tk.X)

    def _build_mit_gain_panel(self, parent):
        # cubemars_joint_names(実機出力対象の関節)はtrajectory_follower_nodeの
        # 起動構成次第で一部の関節だけのことがある(例: root_thetaのみ)ため、
        # 軌道生成パラメータパネルと同じく実際のjoint_names順に読込・適用する
        # (JOINT_NAMES固定でzipするとreal_root_theta_test.launch.py等の1関節構成で
        # 表示が更新されない、2026-08-27発覚のバグと同じ罠を踏むため)。
        frame = tk.LabelFrame(parent, text='MITゲイン (実機CubeMars、trajectory_follower_node)')
        frame.pack(fill=tk.X, pady=(8, 0))

        tk.Label(frame, text='Kp').grid(row=0, column=1)
        tk.Label(frame, text='Kd').grid(row=0, column=2)
        tk.Label(frame, text='torque_ff').grid(row=0, column=3)

        self._mit_joint_names = []
        self.mit_kp_vars = {}
        self.mit_kd_vars = {}
        self.mit_torque_vars = {}
        for i, name in enumerate(JOINT_NAMES):
            tk.Label(frame, text=name).grid(row=i + 1, column=0, sticky='w')
            kp_var = tk.DoubleVar(value=0.0)
            kd_var = tk.DoubleVar(value=0.0)
            tff_var = tk.DoubleVar(value=0.0)
            self.mit_kp_vars[name] = kp_var
            self.mit_kd_vars[name] = kd_var
            self.mit_torque_vars[name] = tff_var
            tk.Entry(frame, textvariable=kp_var, width=6).grid(row=i + 1, column=1, padx=2, pady=2)
            tk.Entry(frame, textvariable=kd_var, width=6).grid(row=i + 1, column=2, padx=2, pady=2)
            tk.Entry(frame, textvariable=tff_var, width=6).grid(row=i + 1, column=3, padx=2, pady=2)

        self.mit_gain_status_label = tk.Label(frame, text='未読込', fg='grey',
                                               wraplength=220, justify=tk.LEFT)
        self.mit_gain_status_label.grid(row=len(JOINT_NAMES) + 1, column=0, columnspan=4, pady=(4, 0))

        btn_row = tk.Frame(frame)
        btn_row.grid(row=len(JOINT_NAMES) + 2, column=0, columnspan=4, pady=4, sticky='we')
        tk.Button(btn_row, text='読込', command=self._on_load_mit_gains).pack(
            side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(btn_row, text='適用', command=self._on_apply_mit_gains,
                  bg='#d9534f', fg='white').pack(side=tk.LEFT, expand=True, fill=tk.X)

    def _build_machine_origin_offset_panel(self, parent):
        frame = tk.LabelFrame(parent, text='機体原点オフセット (soki_sim.urdf.xacro)')
        frame.pack(fill=tk.X, pady=(8, 0))

        tk.Label(frame, text=f'base_link(旋回軸)から実機の機体原点までのズレ[m]。\n'
                              f'ワーク・シューティングボックスもこのオフセットに\n'
                              f'追従して動く(可動範囲: 各軸±{MACHINE_ORIGIN_OFFSET_LIMIT:.2f}m)。',
                 fg='#555', justify=tk.LEFT, wraplength=220).pack(padx=4, pady=(4, 2), anchor='w')

        self.machine_origin_vars = {name: tk.DoubleVar(value=0.0) for name in MACHINE_ORIGIN_JOINT_NAMES}
        entries = tk.Frame(frame)
        entries.pack(padx=4, pady=(0, 2), anchor='w')
        for i, (label, name) in enumerate((
                ('X', 'machine_origin_x_joint'),
                ('Y', 'machine_origin_y_joint'),
                ('Z', 'machine_origin_z_joint'))):
            tk.Label(entries, text=label).grid(row=0, column=i * 2, padx=(0 if i == 0 else 6, 2))
            tk.Entry(entries, textvariable=self.machine_origin_vars[name], width=7).grid(
                row=0, column=i * 2 + 1)

        self.machine_origin_status_label = tk.Label(frame, text='未送信', fg='grey',
                                                      wraplength=220, justify=tk.LEFT)
        self.machine_origin_status_label.pack(padx=4, pady=(2, 4), anchor='w')

        btn_row = tk.Frame(frame)
        btn_row.pack(fill=tk.X, padx=4, pady=(0, 4))
        tk.Button(btn_row, text='現在値を反映', command=self._on_load_machine_origin).pack(
            side=tk.LEFT, expand=True, fill=tk.X)
        tk.Button(btn_row, text='適用', command=self._on_apply_machine_origin,
                  bg='#4a90d9', fg='white').pack(side=tk.LEFT, expand=True, fill=tk.X)

    def _on_load_machine_origin(self):
        if not self.node.has_current_state():
            messagebox.showinfo('未取得', 'まだmixed_joint_statesを受信していません')
            return
        pos = self.node.get_current_positions()
        for name, var in self.machine_origin_vars.items():
            var.set(round(pos.get(name, 0.0), 4))
        self.machine_origin_status_label.config(text='現在値を反映しました', fg='blue')

    def _on_apply_machine_origin(self):
        try:
            raw = {name: self.machine_origin_vars[name].get() for name in MACHINE_ORIGIN_JOINT_NAMES}
        except tk.TclError:
            messagebox.showerror('入力エラー', 'X/Y/Zに数値を入力してください')
            return
        limit = MACHINE_ORIGIN_OFFSET_LIMIT
        clamped = {name: clamp(v, -limit, limit) for name, v in raw.items()}
        for name, v in clamped.items():
            self.machine_origin_vars[name].set(round(v, 4))
        self.node.send_machine_origin(*(clamped[name] for name in MACHINE_ORIGIN_JOINT_NAMES))
        x, y, z = (clamped[name] for name in MACHINE_ORIGIN_JOINT_NAMES)
        text = f'送信しました (x={x:.3f}, y={y:.3f}, z={z:.3f})'
        if any(abs(raw[name] - clamped[name]) > 1e-9 for name in MACHINE_ORIGIN_JOINT_NAMES):
            text += '\n(可動範囲外のためクランプされました)'
        self.machine_origin_status_label.config(text=text, fg='green')

    def _build_origin_panel(self, parent):
        frame = tk.LabelFrame(parent, text='root_theta原点設定 (CubeMars本体、trajectory_follower_node)')
        frame.pack(fill=tk.X, pady=(8, 0))

        tk.Label(frame, text='呼び出し前に、root_theta_jointを原点センサの位置\n'
                              '(真の機械原点)へ物理的に合わせておくこと。',
                 fg='#a00', justify=tk.LEFT, wraplength=220).pack(padx=4, pady=(4, 2), anchor='w')

        self.origin_status_label = tk.Label(frame, text='未実行', fg='grey',
                                             wraplength=220, justify=tk.LEFT)
        self.origin_status_label.pack(padx=4, pady=(0, 4), anchor='w')

        tk.Button(frame, text='/set_root_theta_origin 呼び出し',
                  command=self._on_set_root_theta_origin, bg='#d9534f', fg='white').pack(
            fill=tk.X, padx=4, pady=(0, 4))

    def _on_set_root_theta_origin(self):
        if not messagebox.askyesno(
                '根本θ原点設定の確認',
                'root_theta_jointをCubeMars本体(AK40-10)のフラッシュへ\n'
                '永久原点として書き込みます。\n\n'
                '関節は今、原点センサの位置(真の機械原点)にありますか？\n'
                '間違った位置で実行すると、以後のすべての角度がずれます。'):
            return
        self.origin_status_label.config(text='呼び出し中...', fg='grey')
        ok = self.node.call_trigger_service(
            '/set_root_theta_origin', self._on_set_root_theta_origin_done)
        if not ok:
            self.origin_status_label.config(text='サービス未起動です', fg='red')

    def _on_set_root_theta_origin_done(self, success, message):
        self.origin_status_label.config(
            text=message, fg=('#1a7a1a' if success else 'red'))

    def _build_field_buttons(self, parent):
        field = tk.Frame(parent, padx=8)
        field.pack(fill=tk.X, pady=(0, 8))

        work_frame = tk.LabelFrame(field, text='ワーク (クリックで移動)')
        work_frame.pack(side=tk.LEFT, padx=(0, 8))
        self._build_button_grid(work_frame, [p for row in WORK_POINTS for p in row], header_row=1)

        shoot_frame = tk.LabelFrame(field, text='シューティングボックス (クリックで移動)')
        shoot_frame.pack(side=tk.LEFT)
        self._build_button_grid(shoot_frame, SHOOT_POINTS['L'] + SHOOT_POINTS['R'], header_row=1)

    def _build_button_grid(self, parent, points, header_row=0):
        """points: (label, x, y, z)のリスト。実座標を見た目通りに配置する
        (X昇順=左->右の列、Y降順=奥(ワーク方向)が上->手前が下の行)。"""
        tk.Label(parent, text='← X- ・ X+ →').grid(row=0, column=0, columnspan=99)
        xs = sorted({round(p[1], 6) for p in points})
        ys = sorted({round(p[2], 6) for p in points}, reverse=True)
        for label, x, y, z in points:
            col = xs.index(round(x, 6))
            row = ys.index(round(y, 6)) + header_row
            tk.Button(parent, text=label, width=5,
                      command=lambda x=x, y=y, z=z: self._on_field_point(x, y, z)
                      ).grid(row=row, column=col, padx=2, pady=2)
        tk.Label(parent, text='↑ Y+ (ワーク側) ／ Y- (機体側) ↓').grid(
            row=header_row + len(ys), column=0, columnspan=99)

    def _on_field_point(self, x, y, z):
        self._send_xyz(x, y, z)

    # ---------- realtime state / trajectory params ----------
    def _spin_ros(self):
        # rclpy.spin_once()はmixed_joint_states購読・パラメータサービスの
        # 応答処理に必要(このメソッドの呼び出し=Tkinterのafter()ループ=
        # mainloopと同じメインスレッド上で完結する)。
        rclpy.spin_once(self.node, timeout_sec=0)
        self._refresh_current_state()
        self.after(50, self._spin_ros)

    def _refresh_current_state(self):
        if not self.node.has_current_state():
            return
        pos = self.node.get_current_positions()
        theta = pos['root_theta_joint']
        zj = pos['z_joint']
        r = pos['r_joint']
        x, y, z = joint_to_xyz(theta, zj, r)
        self.current_label.config(text=(
            f'theta={math.degrees(theta):.1f}deg  z_joint={zj:.3f}  r_joint={r:.3f}\n'
            f'X={x:.3f}  Y={y:.3f}  Z={z:.3f}'))
        self._redraw_current_marker(x, y)

    def _redraw_current_marker(self, x, y):
        self.canvas.delete('current')
        cx, cy = self._world_to_canvas(x, y)
        r = 5
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                 outline='#1a7a1a', width=2, tags='current')

    def _on_load_traj_params(self):
        # joint_namesも取得する: trajectory_follower_nodeは実行構成によって
        # JOINT_NAMES(root_theta/z/r)の全部ではなく一部だけで起動されることがある
        # (例: real_root_theta_test.launch.pyはroot_theta_jointのみ)。max_velocity等の
        # 配列は「そのノードの実際のjoint_names順」なので、GUI固定のJOINT_NAMESと
        # 前提にlen比較・zipすると、一致しない構成では表示が更新されず0のままに見える
        # (2026-08-27発覚)。
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME, ['max_velocity', 'max_acceleration', 'control_mode', 'joint_names'],
            self._apply_loaded_traj_params,
            lambda reason: self.traj_status_label.config(text=f'読込失敗: {reason}', fg='red'))
        self.traj_status_label.config(
            text='読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
            fg='grey' if ok else 'red')

    def _apply_loaded_traj_params(self, values):
        vel = values.get('max_velocity')
        accel = values.get('max_acceleration')
        mode = values.get('control_mode')
        names = values.get('joint_names') or JOINT_NAMES
        self._traj_joint_names = list(names)
        missing = [n for n in JOINT_NAMES if n not in names]
        if vel and len(vel) == len(names):
            for name, v in zip(names, vel):
                if name in self.traj_vel_vars:
                    self.traj_vel_vars[name].set(round(v, 4))
        if accel and len(accel) == len(names):
            for name, v in zip(names, accel):
                if name in self.traj_accel_vars:
                    self.traj_accel_vars[name].set(round(v, 4))
        if mode:
            self.mode_var.set(mode)
            self.mode_status_label.config(text=f'現在のモード: {mode}', fg='blue')
        status = '読込完了'
        if missing:
            status += f' (未起動構成: {", ".join(missing)}は表示更新されません)'
        self.traj_status_label.config(text=status, fg='blue')

    def _on_apply_traj_params(self):
        # 実際にtrajectory_follower_nodeが持つjoint_names順で配列を組む
        # (GUI固定のJOINT_NAMESで組むと、一部関節のみの構成では長さ不一致で
        # set_parametersに拒否される。読込未実施時はJOINT_NAMES全部を仮定する)。
        names = getattr(self, '_traj_joint_names', JOINT_NAMES)
        try:
            vel = [self.traj_vel_vars[name].get() for name in names]
            accel = [self.traj_accel_vars[name].get() for name in names]
        except tk.TclError:
            messagebox.showerror('入力エラー', '速度・加速度に数値を入力してください')
            return
        ok = self.node.set_node_params(
            TRAJ_NODE_NAME, {'max_velocity': vel, 'max_acceleration': accel},
            self._apply_traj_set_result)
        self.traj_status_label.config(
            text='適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
            fg='grey' if ok else 'red')

    def _apply_traj_set_result(self, results):
        if results is None:
            self.traj_status_label.config(text='適用に失敗しました(応答なし)', fg='red')
            return
        if all(r.successful for r in results):
            self.traj_status_label.config(text='適用しました', fg='green')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            self.traj_status_label.config(text=f'適用失敗: {reasons}', fg='red')

    def _on_load_mit_gains(self):
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME,
            ['cubemars_kp', 'cubemars_kd', 'cubemars_torque_ff', 'cubemars_joint_names'],
            self._apply_loaded_mit_gains,
            lambda reason: self.mit_gain_status_label.config(text=f'読込失敗: {reason}', fg='red'))
        self.mit_gain_status_label.config(
            text='読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
            fg='grey' if ok else 'red')

    def _apply_loaded_mit_gains(self, values):
        kp = values.get('cubemars_kp')
        kd = values.get('cubemars_kd')
        tff = values.get('cubemars_torque_ff')
        names = values.get('cubemars_joint_names') or []
        self._mit_joint_names = list(names)
        if kp and len(kp) == len(names):
            for name, v in zip(names, kp):
                if name in self.mit_kp_vars:
                    self.mit_kp_vars[name].set(round(v, 4))
        if kd and len(kd) == len(names):
            for name, v in zip(names, kd):
                if name in self.mit_kd_vars:
                    self.mit_kd_vars[name].set(round(v, 4))
        if tff and len(tff) == len(names):
            for name, v in zip(names, tff):
                if name in self.mit_torque_vars:
                    self.mit_torque_vars[name].set(round(v, 4))
        status = '読込完了'
        not_configured = [n for n in JOINT_NAMES if n not in names]
        if not_configured:
            status += f' (実機出力対象外: {", ".join(not_configured)})'
        if not names:
            status = '読込完了 (実機出力対象の関節が設定されていません)'
        self.mit_gain_status_label.config(text=status, fg='blue')

    def _on_apply_mit_gains(self):
        # cubemars_joint_namesの実際の順序(_on_load_mit_gainsで取得済みのもの)で
        # 配列を組む必要がある。未読込のまま適用すると対象関節・順序が不明なため、
        # 先に読込を要求する(誤った関節にゲインを適用する事故を防ぐ)。
        names = getattr(self, '_mit_joint_names', None)
        if not names:
            messagebox.showerror('未読込', '先に「読込」を実行して対象関節を確認してください')
            return
        try:
            kp = [self.mit_kp_vars[name].get() for name in names]
            kd = [self.mit_kd_vars[name].get() for name in names]
            tff = [self.mit_torque_vars[name].get() for name in names]
        except tk.TclError:
            messagebox.showerror('入力エラー', 'Kp/Kd/torque_ffに数値を入力してください')
            return
        if not messagebox.askyesno(
                'MITゲイン適用の確認',
                f'{", ".join(names)} のMITゲインを実機へ即座に反映します。\n'
                'Kpを大きくするほど保持力・応答性が上がりますが、\n'
                '実機にかかる力も大きくなります。よろしいですか？'):
            return
        ok = self.node.set_node_params(
            TRAJ_NODE_NAME,
            {'cubemars_kp': kp, 'cubemars_kd': kd, 'cubemars_torque_ff': tff},
            self._apply_mit_gain_set_result)
        self.mit_gain_status_label.config(
            text='適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
            fg='grey' if ok else 'red')

    def _apply_mit_gain_set_result(self, results):
        if results is None:
            self.mit_gain_status_label.config(text='適用に失敗しました(応答なし)', fg='red')
            return
        if all(r.successful for r in results):
            self.mit_gain_status_label.config(text='適用しました', fg='green')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            self.mit_gain_status_label.config(text=f'適用失敗: {reasons}', fg='red')

    def _on_load_joy_speed(self):
        ok = self.node.request_node_params(
            JOY_NODE_NAME, ['theta_speed', 'z_speed', 'r_speed'],
            self._apply_loaded_joy_speed,
            lambda reason: self.joy_speed_status_label.config(text=f'読込失敗: {reason}', fg='red'))
        self.joy_speed_status_label.config(
            text='読込中...' if ok else 'joy_teleop_nodeに接続できません(use_joy:=trueで起動?)',
            fg='grey' if ok else 'red')

    def _apply_loaded_joy_speed(self, values):
        for name in ('theta_speed', 'z_speed', 'r_speed'):
            if name in values:
                self.joy_speed_vars[name].set(round(values[name], 4))
        self.joy_speed_status_label.config(text='読込完了', fg='blue')

    def _on_apply_joy_speed(self):
        try:
            values = {name: var.get() for name, var in self.joy_speed_vars.items()}
        except tk.TclError:
            messagebox.showerror('入力エラー', '速度に数値を入力してください')
            return
        ok = self.node.set_node_params(JOY_NODE_NAME, values, self._apply_joy_speed_set_result)
        self.joy_speed_status_label.config(
            text='適用中...' if ok else 'joy_teleop_nodeに接続できません(use_joy:=trueで起動?)',
            fg='grey' if ok else 'red')

    def _apply_joy_speed_set_result(self, results):
        if results is None:
            self.joy_speed_status_label.config(text='適用に失敗しました(応答なし)', fg='red')
            return
        if all(r.successful for r in results):
            self.joy_speed_status_label.config(text='適用しました', fg='green')
        else:
            reasons = '; '.join(r.reason for r in results if not r.successful)
            self.joy_speed_status_label.config(text=f'適用失敗: {reasons}', fg='red')

    def _on_mode_changed(self):
        mode = self.mode_var.get()
        ok = self.node.set_node_params(TRAJ_NODE_NAME, {'control_mode': mode}, self._apply_mode_set_result)
        self.mode_status_label.config(
            text='適用中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
            fg='grey' if ok else 'red')

    def _apply_mode_set_result(self, results):
        if results is None or not all(r.successful for r in results):
            reason = results[0].reason if results else '応答なし'
            self.mode_status_label.config(text=f'適用失敗: {reason}', fg='red')
            return
        self.mode_status_label.config(text=f'現在のモード: {self.mode_var.get()}', fg='green')

    def _on_copy_current_to_target(self):
        if not self.node.has_current_state():
            messagebox.showinfo('未取得', 'まだ現在位置を受信していません')
            return
        pos = self.node.get_current_positions()
        x, y, z = joint_to_xyz(pos['root_theta_joint'], pos['z_joint'], pos['r_joint'])
        self.x_var.set(round(x, 3))
        self.y_var.set(round(y, 3))
        self.z_var.set(round(z, 3))

    def _draw_canvas_base(self):
        c = self.CANVAS_SIZE / 2.0
        self.canvas.create_line(0, c, self.CANVAS_SIZE, c, fill='#ccc')
        self.canvas.create_line(c, 0, c, self.CANVAS_SIZE, fill='#ccc')
        px_radius = self._world_to_px_scale() * MAX_RADIUS
        self.canvas.create_oval(c - px_radius, c - px_radius, c + px_radius, c + px_radius,
                                 outline='#4a90d9', dash=(3, 2))
        self.canvas.create_text(c, 10, text='Y+ (ワーク側)', fill='#888')
        self.canvas.create_text(c, self.CANVAS_SIZE - 10, text='Y- (機体後方)', fill='#888')
        self.canvas.create_text(self.CANVAS_SIZE - 28, c, text='X+', fill='#888')
        self.canvas.create_text(28, c, text='X-', fill='#888')
        self.canvas.create_text(c + 14, c + 12, text='機体', fill='#888')

    def _world_to_px_scale(self):
        return (self.CANVAS_SIZE / 2.0 - self.MARGIN) / MAX_RADIUS

    # ---------- coordinate <-> canvas ----------
    def _world_to_canvas(self, x, y):
        c = self.CANVAS_SIZE / 2.0
        s = self._world_to_px_scale()
        return c + x * s, c - y * s

    def _canvas_to_world(self, cx, cy):
        c = self.CANVAS_SIZE / 2.0
        s = self._world_to_px_scale()
        return (cx - c) / s, (c - cy) / s

    def _on_canvas_click(self, event):
        x, y = self._canvas_to_world(event.x, event.y)
        radius = math.hypot(x, y)
        if radius > MAX_RADIUS:
            x *= MAX_RADIUS / radius
            y *= MAX_RADIUS / radius
        self.x_var.set(round(x, 3))
        self.y_var.set(round(y, 3))

    def _redraw_pin(self):
        self.canvas.delete('pin')
        try:
            x, y = self.x_var.get(), self.y_var.get()
        except tk.TclError:
            return
        cx, cy = self._world_to_canvas(x, y)
        r = 5
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill='red', outline='', tags='pin')
        self._update_status()

    def _update_status(self):
        try:
            x, y, z = self.x_var.get(), self.y_var.get(), self.z_var.get()
        except tk.TclError:
            return
        theta, zj, r, clamped = xyz_to_joint(x, y, z)
        text = f'theta={math.degrees(theta):.1f}deg  z_joint={zj:.3f}  r_joint={r:.3f}'
        if clamped:
            text += '\n(可動域外のためクランプされました)'
        self.status_label.config(text=text, fg='red' if clamped else 'blue')

    # ---------- send ----------
    def _on_send(self):
        try:
            x, y, z = self.x_var.get(), self.y_var.get(), self.z_var.get()
        except tk.TclError:
            messagebox.showerror('入力エラー', 'X/Y/Zに数値を入力してください')
            return
        theta, zj, r, _ = xyz_to_joint(x, y, z)
        self.node.send_target(theta, zj, r)
        self._update_status()

    def _on_jog(self, dx, dy):
        try:
            step = self.step_var.get()
            x = self.x_var.get() + dx * step
            y = self.y_var.get() + dy * step
        except tk.TclError:
            messagebox.showerror('入力エラー', 'X/Y/ステップに数値を入力してください')
            return
        self.x_var.set(round(x, 4))
        self.y_var.set(round(y, 4))
        self._on_send()

    def _send_xyz(self, x, y, z):
        self.x_var.set(x)
        self.y_var.set(y)
        self.z_var.set(z)
        self._on_send()

    # ---------- points list ----------
    def _on_add_point(self):
        try:
            x, y, z = self.x_var.get(), self.y_var.get(), self.z_var.get()
        except tk.TclError:
            messagebox.showerror('入力エラー', 'X/Y/Zに数値を入力してください')
            return
        name = simpledialog.askstring('ポイント名', '保存する名前を入力してください',
                                       initialvalue=f'Point{len(self.points) + 1}',
                                       parent=self)
        if not name:
            return
        self.points.append({'name': name, 'x': x, 'y': y, 'z': z})
        self._save_points()
        self._refresh_point_list()

    def _on_delete_point(self):
        sel = self.listbox.curselection()
        if not sel:
            return
        del self.points[sel[0]]
        self._save_points()
        self._refresh_point_list()

    def _on_load_point(self):
        sel = self.listbox.curselection()
        if not sel:
            return
        p = self.points[sel[0]]
        self.x_var.set(p['x'])
        self.y_var.set(p['y'])
        self.z_var.set(p['z'])

    def _on_send_selected(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo('未選択', '一覧からポイントを選択してください')
            return
        p = self.points[sel[0]]
        self._send_xyz(p['x'], p['y'], p['z'])

    def _refresh_point_list(self):
        self.listbox.delete(0, tk.END)
        for p in self.points:
            self.listbox.insert(tk.END, f"{p['name']}  ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f})")

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
    node = CommandGuiNode()
    app = CommandGuiApp(node)
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
