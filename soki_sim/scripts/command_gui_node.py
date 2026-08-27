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
from tkinter import messagebox, simpledialog

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

# soki_sim.urdf.xacro の寸法定数と一致させること
BASE_HEIGHT = 0.06
LIFT_SIZE_Z = 0.08
ARM_LENGTH = 1.244
Z_LOWER, Z_UPPER = 0.0, 0.432
R_LOWER, R_UPPER = -ARM_LENGTH / 2.0, ARM_LENGTH / 2.0

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


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def xyz_to_joint(x, y, z):
    """ワールド座標(X,Y,Z) -> (theta, z_joint, r_joint)。可動域外はクランプする。

    X軸正=右向き、Y軸正=機体からワークに向かう前方。
    root_theta_joint角度は旋回軸に対して定義された内部基準(前方=Y+の時にtheta=0)
    に合わせるため、atan2の引数はatan2(-x, y)となる(X/Yをそのまま使うatan2(y,x)ではない)。
    """
    theta = math.atan2(-x, y)
    radius = math.hypot(x, y)
    raw_r = radius - ARM_LENGTH / 2.0
    raw_z = z - Z_OFFSET
    r = clamp(raw_r, R_LOWER, R_UPPER)
    zj = clamp(raw_z, Z_LOWER, Z_UPPER)
    clamped = (abs(raw_r - r) > 1e-9) or (abs(raw_z - zj) > 1e-9)
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

        self._current_positions = {name: 0.0 for name in JOINT_NAMES}
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
        for var in (self.x_var, self.y_var, self.z_var):
            var.trace_add('write', lambda *_: self._redraw_pin())

        self._build_widgets()
        self._redraw_pin()
        self._refresh_point_list()
        self.after(50, self._spin_ros)

    # ---------- widgets ----------
    def _build_widgets(self):
        main = tk.Frame(self, padx=8, pady=8)
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

        info_col = tk.Frame(main)
        info_col.pack(side=tk.LEFT, padx=(8, 0), fill=tk.Y)
        self._build_current_state_panel(info_col)
        self._build_mode_panel(info_col)
        self._build_trajectory_panel(info_col)
        self._build_joy_speed_panel(info_col)

        self._build_field_buttons()

    def _build_current_state_panel(self, parent):
        frame = tk.LabelFrame(parent, text='現在状態 (リアルタイム)')
        frame.pack(fill=tk.X)
        self.current_label = tk.Label(frame, text='(mixed_joint_states待ち)',
                                       fg='#1a7a1a', justify=tk.LEFT)
        self.current_label.pack(padx=4, pady=4, anchor='w')

    def _build_mode_panel(self, parent):
        frame = tk.LabelFrame(parent, text='動作モード (trajectory_follower_node)')
        frame.pack(fill=tk.X, pady=(8, 0))

        self.mode_var = tk.StringVar(value='auto')
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

    def _build_field_buttons(self):
        field = tk.Frame(self, padx=8)
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
        ok = self.node.request_node_params(
            TRAJ_NODE_NAME, ['max_velocity', 'max_acceleration', 'control_mode'],
            self._apply_loaded_traj_params,
            lambda reason: self.traj_status_label.config(text=f'読込失敗: {reason}', fg='red'))
        self.traj_status_label.config(
            text='読込中...' if ok else 'trajectory_follower_nodeに接続できません(未起動?)',
            fg='grey' if ok else 'red')

    def _apply_loaded_traj_params(self, values):
        vel = values.get('max_velocity')
        accel = values.get('max_acceleration')
        mode = values.get('control_mode')
        if vel and len(vel) == len(JOINT_NAMES):
            for name, v in zip(JOINT_NAMES, vel):
                self.traj_vel_vars[name].set(round(v, 4))
        if accel and len(accel) == len(JOINT_NAMES):
            for name, v in zip(JOINT_NAMES, accel):
                self.traj_accel_vars[name].set(round(v, 4))
        if mode:
            self.mode_var.set(mode)
            self.mode_status_label.config(text=f'現在のモード: {mode}', fg='blue')
        self.traj_status_label.config(text='読込完了', fg='blue')

    def _on_apply_traj_params(self):
        try:
            vel = [self.traj_vel_vars[name].get() for name in JOINT_NAMES]
            accel = [self.traj_accel_vars[name].get() for name in JOINT_NAMES]
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
