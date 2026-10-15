> **cubemars_ctrl版について**: このフォルダは `ros2can/generated_firmware/soki_ak_11` のコピーです。
> 変更点(モード切替をplatformio.iniの環境で行う / モータ1台 / シリアル途絶フェイルセーフ /
> 帰還スロット20-23に受信カウンタ)は `src/config.hpp` 冒頭と、パッケージ直下の README.md を参照。
>
> - 通常運用: `pio run -e cubemars -t upload`
> - CAN ID調査: `pio run -e can_monitor -t upload` → `pio device monitor`
>
> 以下は元のros2canファームのREADME(英語)です。

# xiao_esp32_s3_smd_serial_bridge

## 1. Overview

This firmware targets a XIAO ESP32-S3 based board (with an MCP2561 CAN transceiver) used as either:

- a **standalone serial<->GPIO bridge** (`MODE_IO`), or
- one **node** on a CAN bus (`MODE_CAN`), or
- the **host** that bridges a PC serial link to up to 3 other CAN nodes while also acting as node 0 itself (`MODE_CAN_HOST`), or
- a **read-only CAN sniffer** for bring-up/debugging (`MODE_CAN_MONITOR`), or
- a **dedicated DJI RoboMaster (M3508/M2006/GM6020) driver** for up to 4 motors on its own CAN bus (`MODE_ROBOMAS`, see section 9), or
- a **dedicated CubeMars AK-series (e.g. AK40-10) driver** for up to 4 actuators on its own CAN bus, using the Servo(CAN) and MIT (Force Control) protocols (`MODE_CUBEMARS`, see section 10).

Each board exposes the same local I/O set:

- 3x shared MULTI ports, each configurable per-port as either a digital switch input or a servo PWM output (`MULTI1..3` in `config.hpp`)
- 2x quadrature encoder inputs (ENC1, ENC2), each independently reconfigurable as a brushed-DC motor driver output (PWM+DIR) instead, via `ENC1_MD`/`ENC2_MD` in `config.hpp` (see section 4)

The CAN transport reuses the existing 24-slot int16 serial payload (`Tx_16Data` / `Rx_16Data`) as the common data model; it does not introduce a separate protocol.

---

## 2. Transport Modes

Select exactly one mode in `src/config.hpp`:

- `MODE_IO`: local GPIO/servo/encoder/switch handling over serial only. No CAN.
- `MODE_CAN`: CAN node mode. This board acts as one node on the CAN bus, driven entirely by CAN frames from the host (no serial link to a PC).
- `MODE_CAN_HOST`: CAN host mode. This board owns the PC serial link, relays data to/from up to 3 other CAN nodes, and additionally drives its own local I/O directly (see section 4).
- `MODE_CAN_MONITOR`: passive CAN sniffer. Starts the CAN driver and `canTask` only; no serial bridging and no IO task. Prints one summary line per node to `Serial` whenever all of that node's slots have been observed, for wiring/bring-up checks.
- `MODE_DEBUG`: development/debug mode (PID task).
- `MODE_ROBOMAS`: dedicated DJI RoboMaster driver. Does **not** use `canInit()`/`canTask()` or the node/slot protocol at all — it runs its own CAN bus at 1Mbps speaking DJI's native protocol directly. See section 9.
- `MODE_CUBEMARS`: dedicated CubeMars AK-series driver. Also does **not** use `canInit()`/`canTask()` or the node/slot protocol — it runs its own CAN bus at 1Mbps speaking CubeMars's Servo(CAN) and MIT (Force Control) protocols directly. See section 10.

`main.cpp` enforces that exactly one of `MODE_IO`, `MODE_CAN`, `MODE_CAN_HOST`, `MODE_DEBUG`, `MODE_CAN_MONITOR`, `MODE_ROBOMAS`, `MODE_CUBEMARS` is defined; the build fails otherwise.

---

## 3. CAN Node Addressing

Each board's CAN node index is derived automatically from `CAN_ID`, not set as a separate constant:

```cpp
// CAN_ID is 3 digits: leading digit = bus number, last 2 digits = node number
#define CAN_ID 101
#define CAN_NODE_INDEX ((CAN_ID % 100U) - 1U)
```

So `CAN_ID = 101, 102, 103, 104` map to node index `0, 1, 2, 3`. The host is expected to use `CAN_ID = x01` (node 0); the three CAN node boards on the same bus use `x02`, `x03`, `x04`.

```cpp
#define CAN_NODE_COUNT 4      // max nodes on one bus (host + 3 nodes)
#define CAN_SLOTS_PER_NODE 5  // int16 slots owned by each node
```

Only set `CAN_ID` per board; `CAN_NODE_INDEX`, node addressing, and CAN frame IDs all follow from it.

---

## 3a. Board Variants (`BOARD_VARIANT`)

This same firmware source targets multiple physical boards built around the XIAO ESP32-S3, each with its own pin layout (`src/defs.hpp`) and I/O shape. Select exactly one in `src/config.hpp`:

- `BOARD_SOKI` (default): soki's own board — 3x MULTI (SERVO/SW) ports, 2x ENC/MD ports, `CAN_SLOTS_PER_NODE = 5`. Unchanged from the original design; `MODE_IO` is only implemented for this variant.
- `BOARD_MES`: 2x MD (always dedicated pins, not ENC-shared) + 2x ENC, where ENC2's pins are additionally shared with SW2/SW3 (toggle with `ENC2_SW`). `CAN_SLOTS_PER_NODE = 5` (unchanged).
- `BOARD_SS`: 5x always-on servo outputs + 4x solenoid-valve digital outputs (`TR1-4`, non-zero command = ON), no ENC/MD/SW at all. Needs 9 command channels, so `CAN_SLOTS_PER_NODE = 9` and `CAN_NODE_COUNT = 2` for this variant (see the bounds check in `config.hpp`: `CAN_NODE_COUNT * CAN_SLOTS_PER_NODE` must stay `<= 24`).

`CanIoRxData`/`CanIoTxData` slot layout differs per variant — see the comments in `src/frame_data.hpp` and the per-variant `IO_*` functions in `src/pin_ctrl_task.cpp`. Only `MODE_CAN`/`MODE_CAN_HOST` are implemented for `BOARD_MES`/`BOARD_SS` (a `MODE_IO` build fails to compile for them, since the `Rx_16Data`/`Tx_16Data` direct-index layout below is SOKI-specific).

---

## 4. CAN Slot Mapping

**This section describes `BOARD_SOKI`'s layout.** `BOARD_MES`/`BOARD_SS` use the same node/chunk transport mechanics but a different `CanIoRxData`/`CanIoTxData` slot count and meaning — see §3a above.

The 24-slot payload is divided into 4 node blocks of 5 slots each (20 of the 24 slots are used; the remaining 4 are unused headroom):

| Node | Slot range (of 24) | Board |
|:---|---:|:---|
| Node 0 | 0-4 | Host board itself |
| Node 1 | 5-9 | CAN node board 1 |
| Node 2 | 10-14 | CAN node board 2 |
| Node 3 | 15-19 | CAN node board 3 |

Each node's 5 slots are split into two dedicated I/O arrays (`src/frame_data.hpp`), not addressed directly by slot number:

**Command direction (host -> node), `CanIoRxData[5]`:**

| Index | Meaning |
|---:|:---|
| 0 | SERVO1 angle command (only used if `MULTI1 == 1`) |
| 1 | SERVO2 angle command (only used if `MULTI2 == 1`) |
| 2 | SERVO3 angle command (only used if `MULTI3 == 1`) |
| 3 | MD1 PWM command (only used if `ENC1_MD == 1`): sign = direction, magnitude = duty, clamped to `±MD_PWM_MAX` |
| 4 | MD2 PWM command (only used if `ENC2_MD == 1`): sign = direction, magnitude = duty, clamped to `±MD_PWM_MAX` |

**Feedback direction (node -> host), `CanIoTxData[5]`:**

| Index | Meaning |
|---:|:---|
| 0-2 | SW1-3 switch state (`0` if the corresponding `MULTIx` port is configured as a servo) |
| 3 | ENC1 raw pulse counter value (always `0` if `ENC1_MD == 1`) |
| 4 | ENC2 raw pulse counter value (always `0` if `ENC2_MD == 1`) |

Each 5-slot block is transmitted as two CAN frames (`identifier = 0x100 + node_index*16 + chunk`): chunk 0 carries 4 int16 values, chunk 1 carries the remaining 1 value. `twai_message_t.data` holds each value big-endian.

---

## 5. How Data Flows

### Host mode (`MODE_CAN_HOST`)

1. `serialTask` decodes the 24-slot command payload from the PC into `Rx_16Data`.
2. `canTask` snapshots `Rx_16Data`, applies node 0's slot range directly to the host's own `CanIoRxData` (no CAN round-trip for its own outputs), and sends the other 3 node blocks out over CAN.
3. `canTask` drains CAN feedback frames from the 3 external nodes into a persistent buffer (slots are only overwritten when new frames arrive, so a node's last known value is retained until it reports again), and fills node 0's own slot range directly from the host's local `CanIoTxData` (its own switches/encoders).
4. That merged 24-slot buffer is published to `Tx_16Data` every host loop iteration.
5. `serialTask` sends `Tx_16Data` back to the PC every `CAN_TX_PERIOD_MS` (5 ms).
6. `IO_Task` runs locally on the host exactly as it would on a node, driving SERVO1-3 / MD1-2 outputs from `CanIoRxData` and reading switch/encoder state into `CanIoTxData`.

### Node mode (`MODE_CAN`)

1. `canTask` receives only CAN frames addressed to `CAN_NODE_INDEX` and applies them to local `CanIoRxData`.
2. `IO_Task` drives SERVO1-3 outputs from `CanIoRxData` (for ports configured as servo via `MULTIx`) and MD1-2 outputs from `CanIoRxData` (for channels configured as MD via `ENCx_MD`), and writes SW1-3 / ENC1-2 into `CanIoTxData` (ENC slots read `0` for channels configured as MD).
3. Every `CAN_TX_PERIOD_MS` (5 ms), `CanIoTxData` is packed into this node's slot block and sent back to the host over CAN.

### CAN monitor mode (`MODE_CAN_MONITOR`)

1. `canTask` only receives CAN frames (no serial task, no IO task, no transmit).
2. Frame values are unpacked into a persistent per-node slot buffer, same layout as above.
3. Once every slot for a node has been seen at least once, a summary line (`SW1/SW2/SW3/ENC1/ENC2`) is printed to `Serial` for that node.

---

## 6. Configuration Workflow

1. Open `src/config.hpp`.
2. Set `BOARD_VARIANT` (`BOARD_SOKI`/`BOARD_MES`/`BOARD_SS`, see §3a) to match the physical board being flashed.
3. Set `DEVICE_ID` (serial frame ID, must match the PC-side config for this board).
4. Set `CAN_ID` (3-digit: bus digit + node number, e.g. `101`..`104`). This also determines `CAN_NODE_INDEX`.
5. Choose exactly one mode macro (`MODE_IO`, `MODE_CAN`, `MODE_CAN_HOST`, `MODE_CAN_MONITOR`, or `MODE_DEBUG`). `MODE_IO` is `BOARD_SOKI`-only.
6. `BOARD_SOKI` only: set `MULTI1`/`MULTI2`/`MULTI3` (`0` = switch input, `1` = servo output) to match the wiring.
7. `BOARD_SOKI` only: set `ENC1_MD`/`ENC2_MD` (`0` = encoder input, `1` = MD PWM+DIR output) to match the wiring. `ENCn_A` becomes the MD PWM pin and `ENCn_B` becomes the MD DIR pin when switched to MD. `BOARD_MES` only: set `ENC2_SW` (`0` = ENC2 encoder, `1` = SW2/SW3 switches) to match the wiring.
8. Adjust PWM, servo range, MD PWM frequency/resolution, and pin settings if required.
9. Build and flash with PlatformIO.

---

## 7. Notes / Known Limitations

- The CAN transport intentionally reuses the serial slot model rather than a fully separate protocol.
- Each node has exactly 2 encoder/MD channels (ENC1/MD1, ENC2/MD2, pin-shared per channel via `ENC1_MD`/`ENC2_MD`) and 3 MULTI ports (SW1-3 / SERVO1-3, pin-shared per port via `MULTI1..3`), all reachable over CAN.
- A channel switched to MD (`ENCx_MD == 1`) cannot report encoder feedback at the same time (its `CanIoTxData`/`Tx_16Data` slot always reads `0`); the driver has no feedback (open-loop PWM+DIR) unless a separate sensor is wired elsewhere.
- `MODE_CAN_MONITOR` is read-only and does not drive any outputs; use it to verify wiring/IDs before switching a board to `MODE_CAN` or `MODE_CAN_HOST`.

---

## 9. RoboMaster Driver Mode (`MODE_ROBOMAS`)

Unlike every other mode above, `MODE_ROBOMAS` does not participate in the node/slot
CAN protocol at all. DJI's C620 (M3508), C610 (M2006) and GM6020 controllers speak a
fixed protocol at a fixed **1Mbps** bitrate with fixed CAN IDs that cannot be changed
in firmware. A board in `MODE_ROBOMAS` therefore acts as a **standalone device**: its
own USB-serial link straight to the PC (own `DEVICE_ID`), and its own dedicated CAN
bus with up to `NUM_MOTOR` (4) RoboMaster motors of a **single model** (mixing
M3508/M2006/GM6020 on the same bus is not supported).

`can_task.cpp`'s node/slot bitrate is now also **1Mbps** (`TWAI_TIMING_CONFIG_1MBITS()`),
so a `MODE_CAN`/`MODE_CAN_HOST`/`MODE_CAN_MONITOR` board *can* share the same physical
CAN bus with a `MODE_ROBOMAS` board — the two protocols use non-overlapping CAN ID
ranges (node/slot: `0x100-0x1xx` standard IDs, sized by `CAN_NODE_COUNT`; RoboMaster:
`0x1FE`/`0x200-0x208` standard IDs — see the ID table below). Keep `CAN_NODE_COUNT`
small enough that the node/slot feedback ID range (`0x180 + node*16 + chunk`) doesn't
reach `0x1FE`; the default `CAN_NODE_COUNT` (2-4) is well clear of that.

Bus bandwidth is the real constraint when mixing, not ID collisions — see
[Bus Bandwidth When Mixing Modes](#12-bus-bandwidth-when-mixing-modes-on-one-can-bus)
below. `robomasTask` sends its current command at **200Hz** (`vTaskDelay(5)` in
`robomas.cpp`) rather than 1kHz specifically so it leaves headroom for other traffic
on a shared bus; if this board is on its own dedicated bus with nothing else on it,
this rate is still fine for velocity control.

Select the motor model at compile time in `src/config.hpp`:

```cpp
#define ROBOMAS_MOTOR_TYPE ROBOMAS_MOTOR_M3508 // or ROBOMAS_MOTOR_M2006 / ROBOMAS_MOTOR_GM6020
```

Two control modes are implemented, selectable **per motor, every cycle** via the
`control_mode` slot below:

- **Velocity (mode 0, default)** — unchanged from before. Velocity PID gains
  (`ROBOMAS_KP_VEL` / `ROBOMAS_KI_VEL` / `ROBOMAS_KD_VEL`) are fixed compile-time
  constants in `config.hpp`; they cannot be changed from `ros2can`/the PC side at
  runtime — tune them in firmware and reflash.
- **MIT (mode 1)** — a position PD control loop, symmetric in spirit to CubeMars'
  MIT (Force Control) mode (§10 below), but implemented very differently: DJI's
  C610/C620 ESCs and the GM6020 only ever accept a raw current command over CAN —
  there is no actuator-side position/torque control to hand a dedicated frame to.
  So instead of a new wire protocol to the motor, the PD loop is computed on this
  board (`robomasTask`, 200Hz) and its output current is fed into the exact same
  `sendCurrentCommand()` used by velocity mode. Position feedback comes from the
  RoboMaster's **built-in rotor encoder** (`angle[]`/`vel[]`) — unlike CubeMars
  actuators, there's no way for this board to see an external encoder, so any
  backlash/elasticity in an external belt/pulley transmission is *not* corrected
  for. `mit_kp`/`mit_kd`/`mit_current_ff` are sent fresh from the PC every cycle
  (unlike the velocity mode's fixed gains) — see `config.hpp`'s `ROBOMAS_MIT_*`
  scale constants.

All-zero (the default when nothing is connected / E-STOP) still means mode 0
(velocity) with target 0 — i.e. safe zero-velocity stop, same as before adding MIT.

Slot mapping reuses the standalone 24-slot `Tx_16Data`/`Rx_16Data` frame directly (no
node/slot chunking, since this board is not a node on the host's bus):

**Command (PC -> board), `Rx_16Data`:**

| Index | Meaning |
|---:|:---|
| 0-3 | target for motor 1-4. Velocity mode: raw rpm (output-shaft rpm), no scaling. MIT mode: target position, scale 1 deg/LSB (output-shaft degrees, range ±32767° ≈ ±91 turns — coarser than the 0.1 deg/LSB angle feedback below on purpose, since 0.1 deg/LSB would only reach ±9.1 turns, too little headroom for e.g. soki's z/r axes) |
| 4-7 | control_mode for motor 1-4: 0 = velocity loop (default), 1 = MIT (position PD) |
| 8-11 | MIT mode only: target velocity feed-forward for motor 1-4, scale 1 rpm/LSB (output-shaft rpm) |
| 12-15 | MIT mode only: Kp for motor 1-4, scale `ROBOMAS_MIT_KP_LSB` (A/deg per LSB) |
| 16-19 | MIT mode only: Kd for motor 1-4, scale `ROBOMAS_MIT_KD_LSB` (A/rpm per LSB) |
| 20-23 | MIT mode only: current feed-forward for motor 1-4, scale `ROBOMAS_MIT_CURRENT_FF_LSB_A` (A per LSB) |

**Feedback (board -> PC), `Tx_16Data`:** unchanged, shared by both control modes.

| Index | Meaning |
|---:|:---|
| 0-3 | angle for motor 1-4, output-shaft degrees, scale 0.1 deg/LSB |
| 4-7 | velocity for motor 1-4, output-shaft rpm, no scaling |
| 8-11 | current for motor 1-4, milliamps, scale 0.001 A/LSB |
| 12-23 | unused |

CAN IDs used on the dedicated 1Mbps bus (all fixed by DJI, not configurable):

| Direction | M3508 / M2006 | GM6020 |
|:---|:---|:---|
| Command (group, IDs 1-4) | `0x200` | `0x1FE` |
| Feedback (per motor, ID n) | `0x200 + n` (`0x201`-`0x204`) | `0x204 + n` (`0x205`-`0x208`) |

---

## 10. CubeMars AK-Series Driver Mode (`MODE_CUBEMARS`)

Like `MODE_ROBOMAS`, `MODE_CUBEMARS` does not participate in the node/slot CAN
protocol at all. CubeMars AK-series actuators (e.g. AK40-10) speak the Servo(CAN)
protocol described in the *AK Series Module Product Manual V3.2.0* (section 4.1),
plus the Force Control (MIT) protocol from the same manual (section 4.2), at a
fixed **1Mbps** bitrate. A board in `MODE_CUBEMARS` therefore acts as a **standalone
device**: its own USB-serial link straight to the PC (own `DEVICE_ID`), and its own
dedicated CAN bus with up to `CUBEMARS_MOTOR_COUNT` (4) AK actuators. Unlike DJI's
GM6020 in `MODE_ROBOMAS`, AK actuators run their own onboard closed-loop
position/velocity control (FOC), so this mode does **not** run a host-side PID
loop — it only forwards commands and parses feedback. In MIT mode the actuator
itself computes `torque = Kp*(p_des - p) + Kd*(v_des - v) + t_ff`, so what looks
like a host-side gain (Kp/Kd) is actually just forwarded to the actuator's own
control law every cycle.

`can_task.cpp`'s node/slot bitrate is now also **1Mbps**, so a
`MODE_CAN`/`MODE_CAN_HOST`/`MODE_CAN_MONITOR` board, and/or a `MODE_ROBOMAS` board,
can share the same physical CAN bus with a `MODE_CUBEMARS` board — CubeMars always
uses **extended (29-bit)** IDs (`(control_mode_id << 8) | motor_can_id`, see the ID
table below), which with the default `CUBEMARS_MOTOR_ID_n` values (101-104) land at
`0x300`+, well clear of both the node/slot range and RoboMaster's `0x1FE`/`0x200-0x208`.
`cubemarsTask` sends commands at **200Hz** (`vTaskDelay(5)` in `cubemars.cpp`) for the
same bus-sharing headroom reason as `MODE_ROBOMAS` above — see
[Bus Bandwidth When Mixing Modes](#12-bus-bandwidth-when-mixing-modes-on-one-can-bus).
Note that lowering this doesn't reduce the actuator's own feedback traffic: the
periodic status frame (function ID `0x29`) is broadcast by the actuator on its own
internal timer, independent of how often the host sends commands.

Set each actuator's CAN ID at compile time in `src/config.hpp`, matching the ID
configured on the actuator itself via R-Link/CubeMarsTool:

```cpp
#define CUBEMARS_MOTOR_COUNT 4  // number of actuators on this bus (max 4)
#define CUBEMARS_MOTOR_ID_1 1
#define CUBEMARS_MOTOR_ID_2 2
#define CUBEMARS_MOTOR_ID_3 3
#define CUBEMARS_MOTOR_ID_4 4
```

MIT mode also needs the actuator's encoding range for position/velocity/torque
(`float_to_uint()`'s `x_min`/`x_max` in the manual) to match what the actuator itself
decodes with. These are **not** listed for the AK40-10 in the *AK Series Module
Product Manual V3.2.0* parameter table (only AK10-9/AK60-6/AK70-9/AK80-9/AKE60-8/
AKE80-8 are, and that manual targets the AK 3.0 generation this board's AK40-10 is
built on). The velocity/torque values below instead come from the AK40-10 row of the
*AK Series Module Driver Manual V1.0.18* (AK 2.0 generation) parameter table — the
generation gap is expected to have little effect, but check R-Link (CubeMarsTool)'s
MIT Control tab against the actual unit before relying on them — a mismatch here
silently shifts what a given command value actually means in rad/rad-per-s/N·m:

```cpp
#define CUBEMARS_MIT_P_MIN_RAD -12.5f
#define CUBEMARS_MIT_P_MAX_RAD 12.5f
#define CUBEMARS_MIT_V_MIN_RADPS -45.5f
#define CUBEMARS_MIT_V_MAX_RADPS 45.5f
#define CUBEMARS_MIT_T_MIN_NM -5.0f
#define CUBEMARS_MIT_T_MAX_NM 5.0f
#define CUBEMARS_MIT_KP_MIN 0.0f
#define CUBEMARS_MIT_KP_MAX 500.0f
#define CUBEMARS_MIT_KD_MIN 0.0f
#define CUBEMARS_MIT_KD_MAX 5.0f
```
(Kp/Kd ranges are documented as common across all AK models in both manuals, so
those two can be trusted as-is.)

Each actuator can be commanded in velocity-loop, position-loop, or MIT (Force
Control) mode, selected per actuator per control cycle via a mode slot. Slot mapping
reuses the standalone 24-slot `Tx_16Data`/`Rx_16Data` frame directly (no node/slot
chunking):

**Command (PC -> board), `Rx_16Data`:**

| Index | Meaning |
|---:|:---|
| 0-3 | target for motor 1-4; meaning depends on that motor's `control_mode` slot below |
| 4-7 | `control_mode` for motor 1-4: `0` = velocity loop, `1` = position loop, `2` = MIT (Force Control) |
| 8-11 | MIT target velocity for motor 1-4 (only read when that motor's `control_mode == 2`) |
| 12-15 | MIT Kp for motor 1-4 (only read when `control_mode == 2`) |
| 16-19 | MIT Kd for motor 1-4 (only read when `control_mode == 2`) |
| 20-23 | MIT feed-forward torque for motor 1-4 (only read when `control_mode == 2`) |

When `control_mode == 0` (velocity), the target is electrical speed at **10 ERPM per
LSB** (range approx. ±327670 ERPM), matching the scale the actuator itself uses for
velocity feedback (see below). When `control_mode == 1` (position) or `control_mode
== 2` (MIT), the target is **0.1 deg per LSB** (range ±3276.7°), matching the scale
used for position feedback; for MIT this degree value is converted to radians right
before it's packed into the CAN frame. The MIT-only slots use **0.01 rad/s/LSB**
(velocity), **0.1/LSB** (Kp, range 0-500), **0.01/LSB** (Kd, range 0-5), and **0.01
N·m/LSB** (feed-forward torque) — each is clamped firmware-side to the
`CUBEMARS_MIT_*` range above before being packed.

All-zero `Rx_16Data` (the default when nothing is connected, and what direct-send
E-STOP transmits) decodes to `control_mode = 0` (velocity) with `target = 0` — i.e. a
safe zero-velocity command, not a position jump to zero. No separate E-STOP/disable
logic is needed in firmware for this reason.

**Feedback (board -> PC), `Tx_16Data`:**

| Index | Meaning |
|---:|:---|
| 0-3 | position for motor 1-4, degrees, scale 0.1 deg/LSB |
| 4-7 | speed for motor 1-4, **electrical** rpm (ERPM), scale 10 ERPM/LSB |
| 8-11 | current for motor 1-4, amps, scale 0.01 A/LSB |
| 12-15 | motor temperature for motor 1-4, °C, no scaling |
| 16-19 | error code for motor 1-4 (0=no fault, 1=motor over-temp, 2=over-current, 3=over-voltage, 4=under-voltage, 5=encoder fault, 6=MOSFET over-temp, 7=motor stall) |
| 20-23 | unused |

These are the raw fields of the actuator's periodic status frame (function ID
`0x29`), copied through without conversion. Note that `speed` is **electrical** rpm,
not output-shaft rpm — converting to real rpm requires dividing by the actuator's
pole-pair count and gear ratio, which vary per motor model; this is left to the
`ros2can` GUI profile's per-channel scale factor rather than baked into firmware.

CAN identifiers used on the dedicated 1Mbps bus are extended (29-bit) IDs built as
`(control_mode_id << 8) | motor_can_id`:

| Direction | Control mode ID |
|:---|:---|
| Command: velocity loop | `3` |
| Command: position loop | `4` |
| Command: MIT (Force Control) | `8` |
| Feedback: periodic status | `0x29` |

The MIT command packs Kp(12bit)+Kd(12bit)+Position(16bit)+Speed(12bit)+Torque(12bit)
into the 8-byte payload in that order (manual section 4.2), each value converted with
the same clamp-then-scale-to-unsigned-int transform the manual's `float_to_uint()`
uses. Feedback for MIT-mode motors is unchanged — it's the same periodic status frame
(function ID `0x29`) used by velocity/position loop, since the actuator broadcasts it
on a timer independent of which control mode is active.

Only velocity-loop, position-loop, and MIT commands are implemented; other Servo(CAN)
modes (duty cycle, current loop, current brake, set-origin, position-velocity loop,
motor disable) from the manual are not used by this firmware.

---

## 12. Bus Bandwidth When Mixing Modes on One CAN Bus

`MODE_ROBOMAS`, `MODE_CUBEMARS`, and the node/slot protocol (`MODE_IO`/`MODE_CAN`/
`MODE_CAN_HOST`) can now share one physical 1Mbps CAN bus without CAN ID collisions
(see sections 9/10 above). ID collisions aren't the binding constraint, though —
bus bandwidth is. Ballpark bit cost per 8-byte data frame at 1Mbps, including typical
bit-stuffing overhead:

| Frame type | Approx. time |
|:---|---:|
| Standard ID (11-bit) — used by node/slot and RoboMaster | ~130µs |
| Extended ID (29-bit) — used by CubeMars | ~150µs |

Rough utilization for 2x RoboMaster motors + 2x CubeMars actuators + one node/slot
board (`CAN_NODE_COUNT=2`, `CAN_SLOTS_PER_NODE=5`), assuming each RoboMaster/CubeMars
motor's own feedback frame streams at roughly 1kHz (typical for this class of ESC,
independent of host command rate):

| Source | Command | Feedback | Subtotal |
|:---|---:|---:|---:|
| RoboMaster (200Hz cmd, 2 motors) | ~2.6% | ~26% | ~29% |
| CubeMars (200Hz cmd, 2 motors) | ~6% | ~30% | ~36% |
| Node/slot sensor board | ~10% | ~10% | ~21% |
| **Total** | | | **~86%** |

That's why `robomasTask` and `cubemarsTask` send commands at 200Hz
(`vTaskDelay(5)`) rather than the 1kHz an unshared bus could support at 1ms —
at 1kHz command rate the same mix would exceed 100% of bus capacity (command-side
alone roughly doubles, pushing the total past what the bus can carry, with feedback
traffic unchanged since it's driven by the ESC/actuator's own internal timer, not by
how often the host polls). Even at 200Hz, ~86% leaves limited headroom, so:

- Adding more motors, or a busier sensor board, can push this over 100%.
- Verify on real hardware with `twai_get_status_info()` (see `printHostCanDiagnostics()`
  in `can_task.cpp` for a usage example) — watch the TX error/queue counters rather
  than trusting this estimate alone.
- If it doesn't fit, the fallback is splitting the sensor board onto its own separate
  physical CAN bus (own transceiver wiring, 500kbps is fine there since nothing else
  is on it) rather than trying to shrink the RoboMaster/CubeMars traffic further.

---

## 13. Credits

Developed by NHK Project, RRST, Ritsumeikan University, Japan.
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC
