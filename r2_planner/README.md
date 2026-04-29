# r2_planner — タスク状態管理パッケージ

## 概要

`r2_planner` はロボット R2 の全体タスクシーケンスを管理するパッケージです。  
GUI・コントローラー・自律走行ノードから受け取ったコマンドをもとに現在状態を管理し、  
`nr26_r2_hw_ctrl`（足回り・機構制御）と `r2_autodrive` へ指令を配信します。

---

## ノード: `r2_task_manager`

### 状態一覧

| コード | 名前               | 内容                              |
|-------|--------------------|-----------------------------------|
| 0     | `WAITING`          | 待機・初期状態                    |
| 1     | `ENTER_MFF`        | MFF（マルチフロアフィールド）進入  |
| 2     | `LEAVE_MFF`        | MFF 退出                          |
| 3     | `STAFF_ASSEMBLY`   | スタッフ組み立て                  |
| 4     | `RACK_MOVE`        | ラック移動                        |
| 5     | `STAFF_HAND_TRIGGER` | スタッフハンドトリガー           |

### 遷移モード

| コード | 名前       | 内容                                 |
|--------|-----------|--------------------------------------|
| 0      | `MANUAL`  | 手動遷移（GUIまたはコントローラー操作） |
| 1      | `AUTO`    | 自動遷移（タイマーまたは完了通知で進む） |

---

## トピック接続図

```
┌──────────────────────────────────────────────────────────────┐
│  GUI / rosbridge                                              │
│   r2/task_state        → [Int32]                             │
│   r2/task_color        → [Int32]                             │
│   r2/task_cell         → [Int32]                             │
│   r2/task_transition_mode → [Int32]  (0=MANUAL, 1=AUTO)      │
│   r2/task_command      → [Int32MultiArray]                   │
│   r2/task_state_sequence → [Int32MultiArray]                 │
│   r2/task_state_sequence_names → [String]                    │
│   r2/task_state_pose   → [Float32MultiArray]                 │
│   r2/task_state_mode   → [Int32MultiArray]                   │
│   r2/task_state_odom_reset → [Int32MultiArray]               │
│   r2/task_state_wait_ms → [Int32MultiArray]                  │
│   r2/task_auto_send_enabled → [Bool]                         │
│   r2/task_mff_path     → [Int32MultiArray]                   │
│   r2/task_mff_path_advance → [Bool]                          │
└──────────────┬───────────────────────────────────────────────┘
               │ SUBSCRIBE
               ▼
┌──────────────────────────────────────────────────────────────┐
│              r2_task_manager (r2_planner)                    │
│                                                              │
│  状態管理: state_code / color_code / mff_cell               │
│  シーケンス管理: 状態遷移順リスト                            │
│  MFF パス管理: セル間移動計画                                │
│  自動遷移タイマー (50ms)                                     │
│  ステータス配信タイマー (200ms)                              │
└────┬─────────┬──────────┬──────────┬───────────────────────┘
     │         │          │          │
     ▼         ▼          ▼          ▼
[PUBLISH]  [PUBLISH]  [PUBLISH]  [PUBLISH]
     │         │          │          │
     │    r2/task_status  │    r2/task_status_text
     │    [Int32MultiArray│    [String]
     │    state,color,    │    "state=WAITING color=RED
     │     cell,mode]     │     cell=3 mode=MANUAL"
     │                   │
  r2_drive_mode_cmd    r2_autodrive_cmd
  [Int32MultiArray]    [Float32MultiArray]
  mode=0:MANUAL        [x, y, yaw_rad]
  mode=1:AUTO
  mode=4:MFF
  mode=5:ARENA
     │
     ├─────────────────────────────────────────────────────────┐
     ▼                                                         ▼
┌──────────────────────────┐      ┌───────────────────────────┐
│  R2_SequenceCtrl         │      │  R2_MotionCtrl            │
│  (足回り制御)             │      │  (機構・モノレール制御)    │
│                          │      │                           │
│  SUBSCRIBE:              │      │  SUBSCRIBE:               │
│  r2_drive_mode_cmd ──────┼──────┼──▶ drive_mode_cmd         │
│  r2/task_transition_mode─┼──────┼──▶ transition_mode        │
│                          │      │  r2/task_status_text ─────┼──▶ 機構状態決定
│  manual_mode_=true:      │      │                           │
│   PSボタンで切替          │      │  current_drive_mode_:     │
│   コントローラーで        │      │   MANUAL/AUTO             │
│   足回り直接操作          │      │   PSボタンで切替          │
│                          │      │                           │
│  manual_mode_=false:     │      │  MANUAL時:                │
│   MFF/ARENAシーケンス     │      │   L1/R1: モノレール操作   │
│   自動実行                │      │   UP/DOWN: 機構シーケンス  │
└──────────────────────────┘      └───────────────────────────┘
```

---

## 手動モード / 自動モード

### PSボタンによる切替

PS4コントローラーの **PSボタン** を押すたびに手動/自動モードが切り替わります。

| ノード            | 手動モード (MANUAL)                    | 自動モード (AUTO)                       |
|-----------------|---------------------------------------|----------------------------------------|
| R2_SequenceCtrl | 左スティック+R2で足回り直接操作        | MFF/ARENAシーケンス自動実行             |
| R2_MotionCtrl   | L1/R1でモノレール操作、UP/DOWNで機構操作 | r2_plannerの状態に従い機構を自動動作   |

### r2_planner からの自動切替

`r2/task_transition_mode` トピック (Int32) で外部から切替可能：

```
# 手動モードへ
ros2 topic pub /r2/task_transition_mode std_msgs/msg/Int32 "{data: 0}"

# 自動モードへ
ros2 topic pub /r2/task_transition_mode std_msgs/msg/Int32 "{data: 1}"
```

---

## r2_planner 状態と機構制御の対応

`r2/task_status_text` の `state=` フィールドを `R2_MotionCtrl` がパースして機構動作を決定します。

| r2_planner 状態名       | MotionCtrl 動作               |
|------------------------|-------------------------------|
| `WAITING`              | HEAD_HAND_INIT（始発へ戻す）  |
| `ENTER_MFF`            | HEAD_HAND_PICK_UP_SETTING     |
| `LEAVE_MFF`            | HEAD_HAND_GATTAI_WAITING      |
| `STAFF_ASSEMBLY`       | HEAD_HAND_GATTAI_ASSEMBLY     |
| `KFS_HAND_INIT`        | ハンド初期化（HOME位置）       |
| `KFS_PICK_WAITING`     | 吸着準備位置                  |
| `PICK_UP/MIDDLE/DOWN`  | 高さ選択→吸着                 |
| `KFS_HOLD`             | 吸着保持シーケンス（段階制御） |
| `KFS_MOVE`             | 移動姿勢                      |
| `TTR_SHOOT_MIDDLE`     | 射出姿勢                      |

---

## MFF（多階層フィールド）モード

`r2_drive_mode_cmd` で `mode=4` を送信するとMFFモードに切替わります。

```
r2_task_mff_status [Int32MultiArray]
  [from_cell, to_cell, turn_deg, step_cmd]
        ↓
r2_mff_turn_cmd [Int32]  → R2_SequenceCtrl → MFF旋回シーケンス
r2_mff_step_cmd [Int32]  → R2_SequenceCtrl → 段差上下シーケンス
        ↓
r2_mff_step_complete [Int32]  → r2_task_manager → 自動状態遷移
```

### step_cmd 値

| 値  | 動作                      |
|-----|---------------------------|
| `1` | 壁整列PID → 段差上り       |
| `2` | 段差上り（壁整列なし）     |
| `-1`| 段差下り                  |

---

## パラメータ

| パラメータ                  | デフォルト値  | 説明                      |
|-----------------------------|-------------|---------------------------|
| `initial_state_code`        | `0`         | 起動時の状態コード         |
| `initial_color_code`        | `-1`        | 起動時のチームカラー       |
| `initial_mff_cell`          | `0`         | 起動時のMFFセル番号        |
| `initial_transition_mode`   | `0`         | 起動時の遷移モード (0=手動) |
| `auto_transition_period_ms` | `3000`      | 自動遷移待機時間 [ms]      |
| `initial_auto_send_enabled` | `true`      | 自動送信の初期値           |
| `initial_mff_heading_deg`   | `0`         | MFF初期進行方向 [deg]      |

---

## ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select r2_planner
```

## 起動

```bash
ros2 run r2_planner r2_task_manager
```

または launch ファイルから：

```bash
ros2 launch nr26_r2_hw_ctrl r2_hw_ctrl.launch.py
```
