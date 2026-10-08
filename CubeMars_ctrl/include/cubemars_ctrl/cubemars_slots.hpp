#pragma once
// ESP32(MODE_CUBEMARS)と送受信する24スロット(int16 x24)の割り付けと単位換算。
// ファーム側 firmware/soki_cubemars/src/cubemars.cpp 先頭コメントのスロット表と
// 一対一で対応している(スケールを変える場合は必ず両方を揃えること)。
//
// 指令(ROS -> ESP32), モータm(0-origin)は base + m:
//   0-3:   target       速度ループ: 電気角速度 10ERPM/LSB / MIT: 目標位置 0.1deg/LSB
//   4-7:   control_mode 0=速度ループ, 1=位置ループ, 2=MIT
//   8-11:  MIT 目標速度   0.01rad/s/LSB
//   12-15: MIT Kp         0.1/LSB  (0-500)
//   16-19: MIT Kd         0.01/LSB (0-5)
//   20-23: MIT トルクFF   0.01N·m/LSB
// 帰還(ESP32 -> ROS):
//   0-3:   position     0.1deg/LSB (出力軸)
//   4-7:   speed        10ERPM/LSB (電気角速度。出力軸rpm = ERPM / (極対数 * 減速比))
//   8-11:  current      0.01A/LSB
//   12-15: temperature  degC
//   16-19: error code
//   20-23: 帰還フレーム受信カウンタ(CubeMars_ctrl版ファームで追加)
//
// 全スロット0 = 速度ループ・目標0 = その場停止。起動直後・E-STOP・通信途絶時に
// この状態へ落ちるよう、ファーム/ROSの両方が設計されている。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "cubemars_ctrl/frame_codec.hpp"

namespace cubemars_ctrl
{

constexpr size_t kMaxMotors = 4;

// ---- 指令スロット ----
constexpr size_t kSlotTarget = 0;
constexpr size_t kSlotControlMode = 4;
constexpr size_t kSlotMitVelocity = 8;
constexpr size_t kSlotMitKp = 12;
constexpr size_t kSlotMitKd = 16;
constexpr size_t kSlotMitTorqueFf = 20;

// ---- 帰還スロット ----
constexpr size_t kSlotPosition = 0;
constexpr size_t kSlotSpeed = 4;
constexpr size_t kSlotCurrent = 8;
constexpr size_t kSlotTemperature = 12;
constexpr size_t kSlotError = 16;
constexpr size_t kSlotFeedbackCounter = 20;

// control_modeスロットの値(cubemars.cppのCUBEMARS_MODE_*と一致)
constexpr int16_t kModeVelocity = 0;
constexpr int16_t kModePosition = 1;
constexpr int16_t kModeMit = 2;

// ---- スケール(cubemars.cppと一致) ----
constexpr double kPositionLsbDeg = 0.1;
constexpr double kSpeedLsbErpm = 10.0;
constexpr double kCurrentLsbA = 0.01;
constexpr double kMitVelocityLsbRadps = 0.01;
constexpr double kMitKpLsb = 0.1;
constexpr double kMitKdLsb = 0.01;
constexpr double kMitTorqueLsbNm = 0.01;

// ---- MITモードのレンジ ----
// ファームconfig.hppのCUBEMARS_MIT_*、およびCubeMars AK Series Driver Manualの
// AK40-10行(位置±12.5rad, 速度±45.5rad/s, トルク±5.0N·m)と一致。
// 位置±12.5radはMITプロトコル自体の固定小数点表現の限界で、範囲外はファーム側で
// 無警告にクランプされる(=その角度で止まって動かない)。
constexpr double kPi = 3.14159265358979323846;
constexpr double kMitPositionLimitRad = 12.5;
constexpr double kMitPositionLimitDeg = kMitPositionLimitRad * 180.0 / kPi;  // ≈716.2deg
constexpr double kMitVelocityLimitRadps = 45.5;
constexpr double kMitKpMax = 500.0;
constexpr double kMitKdMax = 5.0;

inline double deg_to_rad(double deg) {return deg * kPi / 180.0;}
inline double rad_to_deg(double rad) {return rad * 180.0 / kPi;}

// スケール済みの実数値をint16スロット値へ丸める(範囲外は飽和させる)。
inline int16_t to_slot(double scaled)
{
  if (!std::isfinite(scaled)) {
    return 0;
  }
  return static_cast<int16_t>(std::lround(std::clamp(scaled, -32768.0, 32767.0)));
}

// 出力軸rpm <-> 電気角速度ERPM。ERPM = 出力軸rpm * 極対数 * 減速比
// (モータ軸rpm = 出力軸rpm * 減速比、電気角 = 機械角 * 極対数)。
inline double output_rpm_to_erpm(double rpm, double pole_pairs, double gear_ratio)
{
  return rpm * pole_pairs * gear_ratio;
}
inline double erpm_to_output_rpm(double erpm, double pole_pairs, double gear_ratio)
{
  return erpm / (pole_pairs * gear_ratio);
}

// 指定モータをゼロ速度指令(その場停止)にする。MIT用スロットも0に戻す。
inline void set_stop(SlotArray & tx, size_t motor)
{
  tx[kSlotTarget + motor] = 0;
  tx[kSlotControlMode + motor] = kModeVelocity;
  tx[kSlotMitVelocity + motor] = 0;
  tx[kSlotMitKp + motor] = 0;
  tx[kSlotMitKd + motor] = 0;
  tx[kSlotMitTorqueFf + motor] = 0;
}

// 速度ループ(Servoモード)で電気角速度erpmを指令する。
inline void set_velocity_erpm(SlotArray & tx, size_t motor, double erpm)
{
  set_stop(tx, motor);
  tx[kSlotTarget + motor] = to_slot(erpm / kSpeedLsbErpm);
}

// MIT(Force Control)モードで位置・速度・Kp・Kdを指令する(トルクFFは0)。
// モータ側で torque = Kp*(pos_des - pos) + Kd*(vel_des - vel) を計算する。
inline void set_mit(
  SlotArray & tx, size_t motor, double pos_deg, double vel_radps, double kp, double kd)
{
  const double pos = std::clamp(pos_deg, -kMitPositionLimitDeg, kMitPositionLimitDeg);
  const double vel = std::clamp(vel_radps, -kMitVelocityLimitRadps, kMitVelocityLimitRadps);
  tx[kSlotTarget + motor] = to_slot(pos / kPositionLsbDeg);
  tx[kSlotControlMode + motor] = kModeMit;
  tx[kSlotMitVelocity + motor] = to_slot(vel / kMitVelocityLsbRadps);
  tx[kSlotMitKp + motor] = to_slot(std::clamp(kp, 0.0, kMitKpMax) / kMitKpLsb);
  tx[kSlotMitKd + motor] = to_slot(std::clamp(kd, 0.0, kMitKdMax) / kMitKdLsb);
  tx[kSlotMitTorqueFf + motor] = 0;
}

struct MotorFeedback
{
  double position_deg{0.0};
  double speed_erpm{0.0};
  double current_a{0.0};
  int temperature_c{0};
  int error_code{0};
  int16_t feedback_counter{0};
};

inline MotorFeedback decode_feedback(const SlotArray & rx, size_t motor)
{
  MotorFeedback fb;
  fb.position_deg = rx[kSlotPosition + motor] * kPositionLsbDeg;
  fb.speed_erpm = rx[kSlotSpeed + motor] * kSpeedLsbErpm;
  fb.current_a = rx[kSlotCurrent + motor] * kCurrentLsbA;
  fb.temperature_c = rx[kSlotTemperature + motor];
  fb.error_code = rx[kSlotError + motor];
  fb.feedback_counter = rx[kSlotFeedbackCounter + motor];
  return fb;
}

// エラーコード(CubeMarsマニュアル Servoモード帰還フレームのData[7])
inline const char * error_text(int code)
{
  switch (code) {
    case 0: return "no fault";
    case 1: return "motor over-temp";
    case 2: return "over-current";
    case 3: return "over-voltage";
    case 4: return "under-voltage";
    case 5: return "encoder fault";
    case 6: return "MOSFET over-temp";
    case 7: return "motor stall";
    default: return "unknown";
  }
}

}  // namespace cubemars_ctrl
