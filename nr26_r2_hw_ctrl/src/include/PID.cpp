#pragma once

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

float PIDController::update(float current, float dt) {
    Error_ = target_ - current;
    Integral_ += (Error_ + last_Error_) * dt / 2; // 台形積分（単純加算でも可？）
    Differential_ = (Error_ - last_Error_) / dt;
    last_Error_ = Error_;

    float output = Kp_ * Error_ + Ki_ * Integral_ + Kd_ * Differential_;
    output = std::clamp(output, -max_out_, max_out_); // アンチワインドアップ
    return output;
}