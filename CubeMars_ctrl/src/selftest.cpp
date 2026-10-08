// 実機なしで実行できる単体チェック。
//   ros2 run CubeMars_ctrl cubemars_selftest
// フレームのエンコード/デコードと、出力軸rpm・deg -> スロット値の換算が
// ファーム(firmware/soki_cubemars/src/cubemars.cpp)の解釈と一致しているかを確かめる。
// ファーム側の式はここに書き写して照合している(片方だけ変えると失敗する)。

#include <cmath>
#include <cstdio>
#include <vector>

#include "cubemars_ctrl/cubemars_slots.hpp"
#include "cubemars_ctrl/frame_codec.hpp"

using namespace cubemars_ctrl;

namespace
{

int g_failures = 0;

void check(bool ok, const char * what)
{
  std::printf("[%s] %s\n", ok ? " OK " : "FAIL", what);
  if (!ok) {
    ++g_failures;
  }
}

bool near(double a, double b, double tol) {return std::abs(a - b) <= tol;}

// ---- ファーム側の解釈(cubemars.cpp sendCommands()より) ----
int32_t firmware_rpm_command_erpm(int16_t target_slot) {return static_cast<int32_t>(target_slot) * 10;}
double firmware_mit_pos_rad(int16_t target_slot) {return target_slot * 0.1 * kPi / 180.0;}
double firmware_mit_vel_radps(int16_t slot) {return slot * 0.01;}
double firmware_mit_kp(int16_t slot) {return slot * 0.1;}
double firmware_mit_kd(int16_t slot) {return slot * 0.01;}

}  // namespace

int main()
{
  // 1) フレームのエンコード -> デコード
  {
    SlotArray data{};
    for (size_t i = 0; i < kSlotCount; ++i) {
      data[i] = static_cast<int16_t>(static_cast<int>(i) * 1000 - 12000);
    }
    const auto frame = encode_frame(11, data);
    check(frame.size() == 52, "フレーム長が52バイト");
    check(frame[0] == 0xAA && frame[1] == 11 && frame[2] == 48, "ヘッダ [0xAA][ID=11][LEN=48]");

    FrameParser parser;
    const std::vector<uint8_t> garbage = {0x00, 0xAA, 0x13, 0xFF};  // 途中から始まったゴミ
    parser.push_bytes(garbage.data(), garbage.size());
    parser.push_bytes(frame.data(), frame.size());
    const auto decoded = parser.pop_frame();
    check(decoded.has_value() && decoded->device_id == 11 && decoded->data == data,
      "ゴミの後ろからでも再同期してデコードできる");

    auto broken = frame;
    broken[10] ^= 0x01;
    FrameParser parser2;
    parser2.push_bytes(broken.data(), broken.size());
    check(!parser2.pop_frame().has_value(), "チェックサム不一致のフレームは捨てる");
  }

  // 2) 速度指令: 出力軸30rpm(極対数14, 減速比10) -> 4200ERPM
  {
    SlotArray tx{};
    set_velocity_erpm(tx, 0, output_rpm_to_erpm(30.0, 14.0, 10.0));
    check(tx[kSlotControlMode] == kModeVelocity, "速度指令でcontrol_mode=0");
    check(firmware_rpm_command_erpm(tx[kSlotTarget]) == 4200, "30rpm -> ファームが4200ERPMとして送信");
    set_velocity_erpm(tx, 0, output_rpm_to_erpm(-30.0, 14.0, 10.0));
    check(firmware_rpm_command_erpm(tx[kSlotTarget]) == -4200, "-30rpm -> -4200ERPM(逆転)");
  }

  // 3) MIT指令: 90deg, 1.5rad/s, Kp=3.6, Kd=1.08
  {
    SlotArray tx{};
    set_mit(tx, 0, 90.0, 1.5, 3.6, 1.08);
    check(tx[kSlotControlMode] == kModeMit, "MIT指令でcontrol_mode=2");
    check(near(firmware_mit_pos_rad(tx[kSlotTarget]), kPi / 2.0, 1e-3), "90deg -> ファームがπ/2 radとして送信");
    check(near(firmware_mit_vel_radps(tx[kSlotMitVelocity]), 1.5, 0.006), "目標速度1.5rad/s");
    check(near(firmware_mit_kp(tx[kSlotMitKp]), 3.6, 0.051), "Kp=3.6");
    check(near(firmware_mit_kd(tx[kSlotMitKd]), 1.08, 0.006), "Kd=1.08");
    check(tx[kSlotMitTorqueFf] == 0, "トルクFF=0");

    set_mit(tx, 0, 5000.0, 0.0, 3.6, 1.08);
    check(near(firmware_mit_pos_rad(tx[kSlotTarget]), 12.5, 0.002), "MIT位置は±12.5radでクランプ");
  }

  // 4) 停止指令は全スロット0(=ros2canのE-STOPと同じ)
  {
    SlotArray tx{};
    set_mit(tx, 0, 90.0, 1.0, 3.6, 1.08);
    set_stop(tx, 0);
    bool all_zero = true;
    for (auto v : tx) {all_zero = all_zero && v == 0;}
    check(all_zero, "停止指令で全スロット0に戻る");
  }

  // 5) 帰還のデコード(ファームはCANの値をスケール変換せずに格納している)
  {
    SlotArray rx{};
    rx[kSlotPosition] = 905;        // 90.5deg
    rx[kSlotSpeed] = 420;           // 4200ERPM
    rx[kSlotCurrent] = -123;        // -1.23A
    rx[kSlotTemperature] = 31;
    rx[kSlotError] = 0;
    rx[kSlotFeedbackCounter] = 77;
    const auto fb = decode_feedback(rx, 0);
    check(near(fb.position_deg, 90.5, 1e-9), "位置 905 -> 90.5deg");
    check(near(erpm_to_output_rpm(fb.speed_erpm, 14.0, 10.0), 30.0, 1e-9), "速度 420 -> 4200ERPM -> 30rpm");
    check(near(fb.current_a, -1.23, 1e-9), "電流 -123 -> -1.23A");
    check(fb.temperature_c == 31 && fb.feedback_counter == 77, "温度・受信カウンタ");
  }

  // 6) int16の範囲外は飽和(折り返して逆回転しない)
  {
    check(to_slot(1e9) == 32767 && to_slot(-1e9) == -32768, "範囲外の値は飽和する");
    check(to_slot(std::nan("")) == 0, "NaNは0(停止側)");
  }

  std::printf("\n%s (%d件失敗)\n", g_failures == 0 ? "すべて成功" : "失敗あり", g_failures);
  return g_failures == 0 ? 0 : 1;
}
