#pragma once
// 24スロット(int16 x24)を最大4台のDamiaoモータに6スロット/台で分配する。
// cubemars_ak_driverプロファイル(ros2can)と同じ「6チャンネル/モータ x 4台
// =24スロット」という割り付け方に揃えてある。
//
// 指令(ROS -> ESP32):
//   local 0: mode_word (0=DISABLED, 1=VEL, 2=POS_VEL)
//            Damiaoモータは明示的なEnable(0xFC)コマンドを送るまでCAN指令を
//            無視する仕様のため、「全ゼロ=DISABLED」を起動直後・通信途絶時の
//            安全な既定値としている(0=VELモードにしてしまうと、ノードが
//            落ちた瞬間に最後の指令のまま動き続ける恐れがあるため)。
//   local 1: target        (0.001単位/LSB。VELモードならrad/s、POS_VELなら rad)
//   local 2: vel_limit     (0.001 rad/s/LSB。POS_VELモードでのみ使用)
// 帰還(ESP32 -> ROS):
//   local 0: motor_state   (モータが返す状態コード。0=Disable, 1=Enable, それ以外=故障。
//                           kMotorState*参照。ros2canのMODE_DAMIAOファームウェアが設定)
//   local 1: fb_counter    (ESP32が帰還フレームを受け取るたびに+1。int16で一周する。
//                           値が進まない=モータから返事が無い(電源OFF・CAN線抜け等))
//   local 3: position      (0.001 rad/LSB。出力軸基準、±12.5rad(P_MAX)の範囲)
//   local 4: velocity      (0.001 rad/s/LSB。出力軸基準。モータ側の分解能は約0.098 rad/s
//                           (V_MAX=200)で0を表せないため、静止中も±0.049程度を示す)
//   local 5: torque        (0.001 N・m/LSB)

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "damiao_ctrl/frame_codec.hpp"

namespace damiao_ctrl
{

constexpr size_t kSlotsPerMotor = 6;
constexpr size_t kMaxMotors = kSlotCount / kSlotsPerMotor;  // 4
constexpr double kScale = 0.001;  // 1 LSB = 0.001 (rad, rad/s, N・m 共通)

enum class DamiaoControlMode : int16_t
{
  kDisabled = 0,
  kVelocity = 1,
  kPositionVelocity = 2,
};

inline int16_t clamp_to_i16(double v)
{
  v = std::round(v);
  v = std::clamp(v, static_cast<double>(INT16_MIN), static_cast<double>(INT16_MAX));
  return static_cast<int16_t>(v);
}

inline size_t motor_slot_offset(size_t motor_index)
{
  return motor_index * kSlotsPerMotor;
}

inline void set_command(
  SlotArray & tx, size_t motor_index, DamiaoControlMode mode, double target, double vel_limit)
{
  const size_t off = motor_slot_offset(motor_index);
  tx[off + 0] = static_cast<int16_t>(mode);
  tx[off + 1] = clamp_to_i16(target / kScale);
  tx[off + 2] = clamp_to_i16(vel_limit / kScale);
}

// モータの状態コード(DM-S2325-1EC説明書の帰還フレームERR欄)。
constexpr int16_t kMotorStateDisabled = 0;
constexpr int16_t kMotorStateEnabled = 1;
constexpr int16_t kMotorStateCommLost = 0xD;  // 通信喪失(モータ側タイムアウト)

inline bool is_motor_fault(int16_t state)
{
  return state != kMotorStateDisabled && state != kMotorStateEnabled;
}

inline const char * motor_state_name(int16_t state)
{
  switch (state) {
    case 0x0: return "Disable";
    case 0x1: return "Enable";
    case 0x5: return "センサ読み取りエラー";
    case 0x6: return "パラメータ読み取りエラー";
    case 0x8: return "過電圧";
    case 0x9: return "低電圧";
    case 0xA: return "過電流";
    case 0xB: return "MOS過熱";
    case 0xC: return "コイル過熱";
    case 0xD: return "通信喪失";
    case 0xE: return "過負荷";
    default: return "不明";
  }
}

struct MotorFeedback
{
  int16_t state{kMotorStateDisabled};
  int16_t counter{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
};

inline MotorFeedback get_feedback(const SlotArray & rx, size_t motor_index)
{
  const size_t off = motor_slot_offset(motor_index);
  MotorFeedback fb;
  fb.state = rx[off + 0];
  fb.counter = rx[off + 1];
  fb.position = static_cast<double>(rx[off + 3]) * kScale;
  fb.velocity = static_cast<double>(rx[off + 4]) * kScale;
  fb.torque = static_cast<double>(rx[off + 5]) * kScale;
  return fb;
}

}  // namespace damiao_ctrl
