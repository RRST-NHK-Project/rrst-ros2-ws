// モータの暴走の見張り(1台分、PC側)。damiao_driver_nodeが使う。
//
// ESP32のファーム(ros2can の firmware/xiao-esp32-s3_can2io/src/runaway_guard.hpp)にも
// 同じ考え方の見張りがあり、そちらが先に止める。これはESP32の見張りが働かなかった時の予備。
//
// 2026-11-09と11-17に、DM3519が指令と逆向きに全速(約−40rad/s)で回り続け、モータの中から
// 白い煙が出た(指令に従わないので、速度の上限では止められない)。次のどちらかが
// kRunawayTimeSec 続いたら「暴走」とみなす。
//   1. 逆向き: 速度制御で、実際の速度が指令と逆向き(または指令がほぼ0)なのに
//      kRunawayMinSpeed より速く、しかも減速していない(kRunawayProgress 下がれば時計を戻す)。
//      正常な減速は約100rad/s²なので引っかからない(11/9の実機の記録で確認)。
//   2. 速すぎる: 実際の速度が上限 + kOverspeedMargin を超えたまま。
#pragma once

#include <chrono>
#include <cmath>

namespace damiao_ctrl
{

class RunawayGuard
{
public:
  using Clock = std::chrono::steady_clock;

  static constexpr double kRunawayMinSpeed = 5.0;   // [rad/s] これ以下の逆向きは見ない
  static constexpr double kRunawayProgress = 1.0;   // [rad/s] これ以上下がれば減速中
  static constexpr double kOverspeedMargin = 5.0;   // [rad/s] 上限をこれだけ超えたら速すぎ
  static constexpr double kRunawayTimeSec = 0.2;    // これだけ続いたら暴走

  void reset()
  {
    wrong_watching_ = false;
    over_watching_ = false;
  }

  // 返事を受け取るたびに呼ぶ。暴走ならtrue。
  //   vel_mode: 速度制御中ならtrue(角度制御ならfalse。逆向きの見張りは速度制御だけ)
  //   target:   速度制御の指令[rad/s](角度制御では使わない)
  //   limit:    速度の上限[rad/s](速度制御はモータの上限、角度制御は速さの上限)
  //   vel:      実際の速度[rad/s]
  bool update(bool vel_mode, double target, double limit, double vel, Clock::time_point now)
  {
    const double speed = std::abs(vel);

    if (speed > limit + kOverspeedMargin) {
      if (!over_watching_) {
        over_watching_ = true;
        over_since_ = now;
      } else if (seconds(over_since_, now) >= kRunawayTimeSec) {
        return true;
      }
    } else {
      over_watching_ = false;
    }

    const bool wrong = vel_mode && speed > kRunawayMinSpeed &&
      (vel * target < 0.0 || std::abs(target) < 0.5);
    if (!wrong) {
      wrong_watching_ = false;
      return false;
    }
    if (!wrong_watching_ || speed < wrong_ref_speed_ - kRunawayProgress) {
      wrong_watching_ = true;  // 見張り始め、または減速している -> 時計を戻す
      wrong_ref_speed_ = speed;
      wrong_since_ = now;
      return false;
    }
    return seconds(wrong_since_, now) >= kRunawayTimeSec;
  }

private:
  static double seconds(Clock::time_point a, Clock::time_point b)
  {
    return std::chrono::duration<double>(b - a).count();
  }

  bool wrong_watching_{false};
  double wrong_ref_speed_{0.0};
  Clock::time_point wrong_since_{};
  bool over_watching_{false};
  Clock::time_point over_since_{};
};

}  // namespace damiao_ctrl
