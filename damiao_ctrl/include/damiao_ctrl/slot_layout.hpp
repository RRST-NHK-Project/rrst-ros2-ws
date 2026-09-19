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
//   local 3: position      (0.001 rad/LSB)
//   local 4: velocity      (0.001 rad/s/LSB)
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

struct MotorFeedback
{
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
};

inline MotorFeedback get_feedback(const SlotArray & rx, size_t motor_index)
{
  const size_t off = motor_slot_offset(motor_index);
  MotorFeedback fb;
  fb.position = static_cast<double>(rx[off + 3]) * kScale;
  fb.velocity = static_cast<double>(rx[off + 4]) * kScale;
  fb.torque = static_cast<double>(rx[off + 5]) * kScale;
  return fb;
}

}  // namespace damiao_ctrl
