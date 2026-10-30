#pragma once
// ros2canのserial_tx_<ID>/serial_rx_<ID>(int16 x24)を、RobStride EduLite 05 用に
// 8スロット/モータ x 最大2台 + 診断4スロット(20-23)へ割り付ける。
// ESP32側(ros2canファームのrobstride.cpp先頭コメント)と必ず一致させること。
//
// 指令(ROS -> ESP32):
//   local 0: mode_word (0=DISABLED, 1=VEL(速度), 2=POS(位置CSP))
//            「全ゼロ=DISABLED」を起動直後・通信途絶時の安全な既定値にしている。
//   local 1,2: target int32 (1が上位16bit, 2が下位16bit)。0.001単位
//              (VELならrad/s、POSならrad)。位置は多回転(±数千rad)を扱うためint32。
//   local 3: vel_limit (0.01 rad/s/LSB。POSモードの速度上限。0以下だとESP32は有効化しない)
// 帰還(ESP32 -> ROS):
//   local 0: motor_mode  (0=Reset(停止), 1=校正, 2=運転)
//   local 1: fault_flags (下位6bit=モータの故障、0x100=ESP32が止めた(ラッチ)、
//                         0x200=位置をまだ読めていない)
//   local 2: fb_counter  (帰還を受け取るたびに+1。止まったらモータから返事が無い)
//   local 3,4: position int32 (3が上位16bit) [0.001 rad/LSB] 出力軸の多回転位置(mechPos)
//   local 5: velocity [0.01 rad/s/LSB]
//   local 6: torque   [0.001 N・m/LSB]
//   local 7: 温度     [0.1 degC/LSB]
// 診断(ESP32 -> ROS): slot 20 = IDスキャンで応答したモータの数、21-23 = そのCAN ID(無ければ-1)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace robstride_ctrl
{

constexpr size_t kSlotCount = 24;
using SlotArray = std::array<int16_t, kSlotCount>;

constexpr size_t kSlotsPerMotor = 8;
constexpr size_t kMaxMotors = 2;
constexpr size_t kDiagSlot = 20;
constexpr size_t kDiagIdSlots = 3;

constexpr double kTargetScale = 0.001;   // rad または rad/s
constexpr double kPositionScale = 0.001; // rad/LSB
constexpr double kVelocityScale = 0.01;  // rad/s/LSB
constexpr double kTorqueScale = 0.001;   // N・m/LSB
constexpr double kTempScale = 0.1;       // degC/LSB

enum class RobstrideControlMode : int16_t
{
  kDisabled = 0,
  kVelocity = 1,
  kPosition = 2,
};

constexpr int16_t kMotorModeReset = 0;
constexpr int16_t kMotorModeCali = 1;
constexpr int16_t kMotorModeRun = 2;

constexpr uint16_t kFaultMask = 0x3F;
constexpr uint16_t kFlagLatched = 0x100;
constexpr uint16_t kFlagPositionUnknown = 0x200;

inline int16_t clamp_to_i16(double v)
{
  v = std::round(v);
  v = std::clamp(v, static_cast<double>(INT16_MIN), static_cast<double>(INT16_MAX));
  return static_cast<int16_t>(v);
}

inline int32_t clamp_to_i32(double v)
{
  v = std::round(v);
  v = std::clamp(v, static_cast<double>(INT32_MIN), static_cast<double>(INT32_MAX));
  return static_cast<int32_t>(v);
}

inline void split_i32(int32_t value, int16_t & hi, int16_t & lo)
{
  const auto u = static_cast<uint32_t>(value);
  hi = static_cast<int16_t>(static_cast<uint16_t>(u >> 16));
  lo = static_cast<int16_t>(static_cast<uint16_t>(u & 0xFFFF));
}

inline int32_t join_i32(int16_t hi, int16_t lo)
{
  const uint32_t u = (static_cast<uint32_t>(static_cast<uint16_t>(hi)) << 16) |
    static_cast<uint32_t>(static_cast<uint16_t>(lo));
  return static_cast<int32_t>(u);
}

inline size_t motor_slot_offset(size_t motor_index)
{
  return motor_index * kSlotsPerMotor;
}

inline void set_command(
  SlotArray & tx, size_t motor_index, RobstrideControlMode mode, double target, double vel_limit)
{
  const size_t off = motor_slot_offset(motor_index);
  tx[off + 0] = static_cast<int16_t>(mode);
  split_i32(clamp_to_i32(target / kTargetScale), tx[off + 1], tx[off + 2]);
  tx[off + 3] = clamp_to_i16(vel_limit / kVelocityScale);
}

// 故障ビット(下位6bit)の意味。EL05説明書CN版の通信タイプ2(帰還)のID bit16-21。
inline std::vector<const char *> fault_names(uint16_t flags)
{
  static const char * kNames[6] = {
    "低電圧", "過電流", "過温", "磁気エンコーダ故障", "ストール過負荷", "未校正"};
  std::vector<const char *> out;
  for (int bit = 0; bit < 6; ++bit) {
    if (flags & (1u << bit)) {
      out.push_back(kNames[bit]);
    }
  }
  return out;
}

inline const char * motor_mode_name(int16_t mode)
{
  switch (mode) {
    case kMotorModeReset: return "停止";
    case kMotorModeCali: return "校正中";
    case kMotorModeRun: return "運転";
    default: return "不明";
  }
}

struct MotorFeedback
{
  int16_t mode{kMotorModeReset};
  uint16_t flags{0};
  int16_t counter{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double temperature{0.0};
};

inline MotorFeedback get_feedback(const SlotArray & rx, size_t motor_index)
{
  const size_t off = motor_slot_offset(motor_index);
  MotorFeedback fb;
  fb.mode = rx[off + 0];
  fb.flags = static_cast<uint16_t>(rx[off + 1]);
  fb.counter = rx[off + 2];
  fb.position = static_cast<double>(join_i32(rx[off + 3], rx[off + 4])) * kPositionScale;
  fb.velocity = static_cast<double>(rx[off + 5]) * kVelocityScale;
  fb.torque = static_cast<double>(rx[off + 6]) * kTorqueScale;
  fb.temperature = static_cast<double>(rx[off + 7]) * kTempScale;
  return fb;
}

}  // namespace robstride_ctrl
