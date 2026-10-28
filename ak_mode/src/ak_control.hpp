// ak_control.hpp
// CubeMars AK (xiao-esp32-s3_can2io, MODE_CUBEMARS) 用の指令生成ロジック。ROS2 に依存しない。
//
// スロット割当 (cubemars.cpp):
//   指令 serial_tx (本機 Rx_16Data): [m]=target, [4+m]=control_mode (0=速度,1=位置)
//        速度: 10ERPM/LSB   位置: 0.1deg/LSB
//   帰還 serial_rx (本機 Tx_16Data): [m]=位置(0.1deg) [4+m]=速度(10ERPM) [8+m]=電流(0.01A)
//                                    [12+m]=温度(degC) [16+m]=エラーコード
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace ak {

constexpr int kSlots = 24;
constexpr int kModeOffset = 4;

enum class Mode { Velocity, Position };

struct ControlParams {
    int motor_index = 0;        // 0-3 (モータ1-4)
    double max_erpm = 3000.0;   // 速度モード: スティック全開の電気角速度 [ERPM]
    double accel_erpm = 6000.0; // 速度モード: 変化率の上限 [ERPM/s]
    double max_dps = 90.0;      // 位置モード: スティック全開の目標移動速度 [deg/s]
    double step_deg = 90.0;     // 位置モード: D-pad 1回で動かす量 [deg]
    double deadzone = 0.1;      // スティックのデッドゾーン
};

struct Feedback {
    double pos_deg = 0.0;
    double speed_erpm = 0.0;
    double current_a = 0.0;
    int temp_c = 0;
    int error = 0;
};

class Controller {
public:
    explicit Controller(const ControlParams& p) : p_(p) {}

    // ---- 入力 ----
    void setStick(double v) { stick_ = v; }          // 上 = +1
    void setDeadman(bool d) { deadman_ = d; }
    void setFeedback(const int16_t* tx) {            // tx は kSlots 個以上
        const int m = p_.motor_index;
        fb_.pos_deg = tx[m] * 0.1;
        fb_.speed_erpm = tx[4 + m] * 10.0;
        fb_.current_a = tx[8 + m] * 0.01;
        fb_.temp_c = tx[12 + m];
        fb_.error = tx[16 + m];
        has_fb_ = true;
    }

    // ---- ボタン操作 ----
    void selectVelocity() { mode_ = Mode::Velocity; vel_cmd_ = 0.0; }
    bool selectPosition() {                          // フィードバック未受信なら false
        if (!has_fb_) return false;
        target_deg_ = fb_.pos_deg;
        mode_ = Mode::Position;
        return true;
    }
    bool syncTarget() {                              // 目標を現在位置へ
        if (mode_ != Mode::Position || !has_fb_) return false;
        target_deg_ = fb_.pos_deg;
        return true;
    }
    bool stepTarget(int dir) {                       // D-pad: 目標を ±step_deg
        if (mode_ != Mode::Position || !deadman_) return false;
        target_deg_ = clampDeg(target_deg_ + dir * p_.step_deg);
        return true;
    }

    // ---- 指令生成 (周期的に呼ぶ) ----
    void update(double dt, std::array<int16_t, kSlots>& out) {
        out.fill(0);
        const int m = p_.motor_index;
        const double s = applyDeadzone(stick_);

        if (mode_ == Mode::Velocity) {
            const double desired = deadman_ ? s * p_.max_erpm : 0.0;
            const double step = p_.accel_erpm * dt;
            vel_cmd_ += std::min(std::max(desired - vel_cmd_, -step), step);
            out[m] = clamp16(vel_cmd_ / 10.0);
            out[kModeOffset + m] = 0;
        } else {
            if (deadman_) target_deg_ = clampDeg(target_deg_ + s * p_.max_dps * dt);
            out[m] = clamp16(target_deg_ * 10.0);
            out[kModeOffset + m] = 1;
        }
    }

    // ---- 状態参照 ----
    Mode mode() const { return mode_; }
    bool hasFeedback() const { return has_fb_; }
    bool deadman() const { return deadman_; }
    double velCmdErpm() const { return vel_cmd_; }
    double targetDeg() const { return target_deg_; }
    const Feedback& feedback() const { return fb_; }

private:
    static int16_t clamp16(double v) {
        return static_cast<int16_t>(std::lround(std::min(std::max(v, -32767.0), 32767.0)));
    }
    static double clampDeg(double d) { return std::min(std::max(d, -3200.0), 3200.0); }  // int16 の 0.1deg 範囲内
    double applyDeadzone(double x) const {
        if (std::fabs(x) < p_.deadzone) return 0.0;
        return (x - std::copysign(p_.deadzone, x)) / (1.0 - p_.deadzone);
    }

    ControlParams p_;
    Mode mode_ = Mode::Velocity;
    double stick_ = 0.0;
    bool deadman_ = false;
    double vel_cmd_ = 0.0;
    double target_deg_ = 0.0;
    Feedback fb_;
    bool has_fb_ = false;
};

}  // namespace ak
