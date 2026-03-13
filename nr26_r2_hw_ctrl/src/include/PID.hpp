#pragma once

#include <algorithm>

class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd, float max_out);
    void set_target(float target);
    void set_gain(float Kp, float Ki, float Kd);
    float update(float current, float dt);

private:
    float Kp_;
    float Ki_;
    float Kd_;

    float max_out_;
    float target_;

    float Error_;
    float last_Error_;
    float Integral_;
    float Differential_;
    float target_;
}