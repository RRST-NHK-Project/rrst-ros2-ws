/*
PID制御ライブラリ
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#pragma once

#include <algorithm>

class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd, float max_out)
        : Kp_(Kp),
          Ki_(Ki),
          Kd_(Kd),
          max_out_(max_out),
          target_(0.0f),
          Error_(0.0f),
          last_Error_(0.0f),
          last_current_(0.0f),
          Integral_(0.0f),
          Differential_(0.0f) {}

    void set_target(float target) {
        target_ = target;
    }

    void set_gain(float Kp, float Ki, float Kd) {
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }

    void reset() {
        Error_ = 0.0f;
        last_Error_ = 0.0f;
        last_current_ = 0.0f;
        Integral_ = 0.0f;
        Differential_ = 0.0f;
    }

    float update(float current, float dt) {
        dt = std::max(dt, 0.0001f);

        Error_ = target_ - current;

        if (Ki_ != 0.0f) {
            Integral_ += (Error_ + last_Error_) * dt / 2;
            float integral_limit = max_out_ / std::abs(Ki_);
            Integral_ = std::clamp(Integral_, -integral_limit, integral_limit);
        } else {
            Integral_ = 0.0f;
        }

        Differential_ = -(current - last_current_) / dt;
        last_current_ = current;

        last_Error_ = Error_;

        float output = Kp_ * Error_ + Ki_ * Integral_ + Kd_ * Differential_;
        return std::clamp(output, -max_out_, max_out_);
    }

private:
    float Kp_;
    float Ki_;
    float Kd_;

    float max_out_;
    float target_;

    float Error_;
    float last_Error_;
    float last_current_;
    float Integral_;
    float Differential_;
};