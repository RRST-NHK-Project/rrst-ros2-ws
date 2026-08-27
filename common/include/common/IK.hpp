/*
IKライブラリ
Copyright (c) 2026 RRST-NHK-Project. All rights reserved.
*/

#pragma once

#include <algorithm>
#include <cmath>

/*
円筒座標(r-θ-z)機構の逆運動学。
目標手先位置を直交座標[m]で受け取り、各軸の指令値に変換する。
旋回軸には回転制限がある前提なので、可動域外は到達不可として通知したうえでクランプする。
*/
class IKController {
public:
    // 各軸の可動域を与えて生成する
    IKController(float r_min, float r_max, float theta_min, float theta_max, float z_min, float z_max)
        : r_min_(r_min),
          r_max_(r_max),
          theta_min_(theta_min),
          theta_max_(theta_max),
          z_min_(z_min),
          z_max_(z_max),

          X_(0.0f),
          Y_(0.0f),
          Z_(0.0f),

          out_r_(0.0f),
          out_theta_(0.0f),
          out_z_(0.0f),

          reachable_(true) {}

    // 目標手先位置を直交座標[m]で与える
    void set_target(float X, float Y, float Z) {
        X_ = X;
        Y_ = Y;
        Z_ = Z;
    }

    void reset() {
        X_ = 0.0f;
        Y_ = 0.0f;
        Z_ = 0.0f;
        out_r_ = 0.0f;
        out_theta_ = 0.0f;
        out_z_ = 0.0f;
        reachable_ = true;
    }

    // 逆運動学を解く。目標が可動域内に収まっていればtrueを返す
    bool solve() {
        reachable_ = true;

        // 半径: x-y平面上の原点からの距離
        float r = std::hypot(X_, Y_);

        // 旋回角: 旋回中心の近傍ではθが定まらないので直前の角度を保持する(特異点処理)
        float theta;
        if (r < kSingularRadius) {
            theta = out_theta_;
            reachable_ = false;
        } else {
            theta = std::atan2(Y_, X_);
        }

        // 可動域外なら到達不可を通知する。クランプ後の値は目標とずれる点に注意
        if (r < r_min_ || r > r_max_) {
            reachable_ = false;
        }
        if (theta < theta_min_ || theta > theta_max_) {
            reachable_ = false;
        }
        if (Z_ < z_min_ || Z_ > z_max_) {
            reachable_ = false;
        }

        out_r_ = std::clamp(r, r_min_, r_max_);
        out_theta_ = std::clamp(theta, theta_min_, theta_max_);
        out_z_ = std::clamp(Z_, z_min_, z_max_);

        return reachable_;
    }

    float get_r() const {
        return out_r_;
    }

    float get_theta() const {
        return out_theta_;
    }

    float get_z() const {
        return out_z_;
    }

    bool reachable() const {
        return reachable_;
    }

    // 順運動学。IKの結果を戻して往復テストする用
    static void forward(float r, float theta, float z, float &X, float &Y, float &Z) {
        X = r * std::cos(theta);
        Y = r * std::sin(theta);
        Z = z;
    }

private:
    static constexpr float kSingularRadius = 1e-3f;  // これ未満の半径はθ不定とみなす [m]

    float r_min_;
    float r_max_;
    float theta_min_;
    float theta_max_;
    float z_min_;
    float z_max_;

    float X_;  // 目標手先位置 [m]
    float Y_;  // 目標手先位置 [m]
    float Z_;  // 目標手先位置 [m]

    float out_r_;      // 半径方向の指令値 [m]
    float out_theta_;  // 旋回角の指令値 [rad]
    float out_z_;      // 昇降の指令値 [m]

    bool reachable_;
};
