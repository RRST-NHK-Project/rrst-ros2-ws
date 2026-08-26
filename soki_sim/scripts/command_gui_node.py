#!/usr/bin/env python3
"""
soki_sim: 手先の目標位置(X,Y,Z)を指定して指令を送るGUIノード。

座標系はbase_link原点を基準としたワールド座標(Z軸は鉛直上向き)。
XY平面上でクリックしてピンを置く/数値入力で目標位置を指定し、
逆運動学でroot_theta_joint・z_joint・r_jointに変換して
/mixed_joint_states (joint_state_publisher(_gui)のsource_list) へpublishする。
motor_mixer_nodeが/joint_states経由でこれを検知し、motor1/motor2側の値も
自動的に追従計算される(launch/display.launch.py参照)。

以下の寸法定数はsoki_sim.urdf.xacroの値と対応させているため、
xacro側を変更した場合はこちらも合わせて変更すること。
  base_height, lift_size_z, arm_length, z_lower/upper, r_lower/upper

複数の目標位置は名前を付けて保存でき、~/.config/soki_sim/points.json に
自動保存される(次回起動時に読み込まれる)。

tkinterが必要(未インストールの場合: sudo apt install python3-tk)。
"""
import json
import math
import os
import tkinter as tk
from tkinter import messagebox, simpledialog

import rclpy
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

JOINT_NAMES = ['root_theta_joint', 'z_joint', 'r_joint']


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def xyz_to_joint(x, y, z):
    """ワールド座標(X,Y,Z) -> (theta, z_joint, r_joint)。可動域外はクランプする。"""
    theta = math.atan2(y, x)
    radius = math.hypot(x, y)
    raw_r = radius - ARM_LENGTH / 2.0
    raw_z = z - Z_OFFSET
    r = clamp(raw_r, R_LOWER, R_UPPER)
    zj = clamp(raw_z, Z_LOWER, Z_UPPER)
    clamped = (abs(raw_r - r) > 1e-9) or (abs(raw_z - zj) > 1e-9)
    return theta, zj, r, clamped


class CommandGuiNode(Node):

    def __init__(self):
        super().__init__('command_gui_node')
        self.pub_ = self.create_publisher(JointState, 'mixed_joint_states', 10)

    def send_target(self, theta, zj, r):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [theta, zj, r]
        self.pub_.publish(msg)


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

    # ---------- widgets ----------
    def _build_widgets(self):
        main = tk.Frame(self, padx=8, pady=8)
        main.pack(fill=tk.BOTH, expand=True)

        left = tk.Frame(main)
        left.pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(left, text='XY平面 (クリックでピン設置)').pack()
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

        jog_frame = tk.LabelFrame(mid, text='ジョグ (X,Y)')
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

    def _draw_canvas_base(self):
        c = self.CANVAS_SIZE / 2.0
        self.canvas.create_line(0, c, self.CANVAS_SIZE, c, fill='#ccc')
        self.canvas.create_line(c, 0, c, self.CANVAS_SIZE, fill='#ccc')
        px_radius = self._world_to_px_scale() * MAX_RADIUS
        self.canvas.create_oval(c - px_radius, c - px_radius, c + px_radius, c + px_radius,
                                 outline='#4a90d9', dash=(3, 2))

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
