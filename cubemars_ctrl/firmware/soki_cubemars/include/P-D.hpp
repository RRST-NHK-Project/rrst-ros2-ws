/*
微分先行型P-D制御ライブラリ
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#pragma once

#include <algorithm>

class PDController {
public:
    PDController(float Kp, float Kd, float max_out)
        : Kp_(Kp),
          Kd_(Kd),
          max_out_(max_out),
          target_(0.0f),
          Error_(0.0f),
          last_current_(0.0f),
          Differential_(0.0f) {}

    void set_target(float target) {
        target_ = target;
    }

    void set_gain(float Kp, float Kd) {
        Kp_ = Kp;
        Kd_ = Kd;
    }

    void reset() {
        Error_ = 0.0f;
        last_current_ = 0.0f;
        Differential_ = 0.0f;
    }

    float update(float current, float dt) {
        dt = std::max(dt, 0.0001f);

        Error_ = target_ - current;
        Differential_ = -(current - last_current_) / dt;
        last_current_ = current;

        float output = Kp_ * Error_ + Kd_ * Differential_;
        return std::clamp(output, -max_out_, max_out_);
    }

private:
    float Kp_;
    float Kd_;

    float max_out_;
    float target_;

    float Error_;
    float last_current_;
    float Differential_;
};