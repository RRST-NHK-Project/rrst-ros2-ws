#include "PID.hpp"
#include <algorithm>

PIDController::PIDController(float Kp, float Ki, float Kd, float max_out) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    max_out_ = max_out;
    target_ = 0.0;
    Error_ = 0.0;
    last_Error_ = 0.0;
    last_current_ = 0.0;
    Integral_ = 0.0;
    Differential_ = 0.0;
}

void PIDController::set_target(float target) {
    target_ = target;
}

void PIDController::set_gain(float Kp, float Ki, float Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void PIDController::reset() {
    Error_ = 0.0;
    last_Error_ = 0.0;
    last_current_ = 0.0;
    Integral_ = 0.0;
    Differential_ = 0.0;
}

float PIDController::update(float current, float dt) {

    dt = std::max(dt, 0.0001f); // dtが0のときの0除算防止

    Error_ = target_ - current;

    if (Ki_ != 0.0f) {
        Integral_ += (Error_ + last_Error_) * dt / 2;                       // 台形積分
        float integral_limit = max_out_ / std::abs(Ki_);                    // 積分上限の逆算
        Integral_ = std::clamp(Integral_, -integral_limit, integral_limit); // 積分出力制限
    } else {
        Integral_ = 0.0f; // Kiが0のときは積分をリセット
    }

    Differential_ = -(current - last_current_) / dt;
    last_current_ = current;

    last_Error_ = Error_;

    float output = Kp_ * Error_ + Ki_ * Integral_ + Kd_ * Differential_;
    output = std::clamp(output, -max_out_, max_out_); // 出力制限
    return output;
}