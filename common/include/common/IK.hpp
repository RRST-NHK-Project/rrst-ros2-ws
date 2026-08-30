/*
IKライブラリ
Copyright (c) 2026 RRST-NHK-Project. All rights reserved.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

/*
円筒座標(r-θ-z)機構の逆運動学。
目標手先位置を直交座標[m]で受け取り、旋回角θと差動2モータ角zr1/zr2に変換する。

r方向とz方向は2つのモータの差動で駆動される:
    z = a * (zr1 + zr2)   同方向に同じ角度で昇降のみ
    r = b * (zr1 - zr2)   逆方向に同じ角度で伸縮のみ
逆に解くと zr1 = (z/a + r/b)/2, zr2 = (z/a - r/b)/2 となる。

旋回軸には回転制限がある前提なので、可動域外は到達不可として通知したうえでクランプする。
*/
class IKController {
public:
    // 各軸の可動域と差動の機構定数a,bを与えて生成する
    IKController(float r_min, float r_max,
                 float theta_min, float theta_max,
                 float z_min, float z_max,
                 float a, float b)
        : r_min_(r_min),
          r_max_(r_max),
          theta_min_(theta_min),
          theta_max_(theta_max),
          z_min_(z_min),
          z_max_(z_max),

          a_(a),
          b_(b),
          zr_min_(-std::numeric_limits<float>::infinity()),
          zr_max_(std::numeric_limits<float>::infinity()),

          X_(0.0f),
          Y_(0.0f),
          Z_(0.0f),

          out_r_(0.0f),
          out_theta_(0.0f),
          out_z_(0.0f),
          out_zr1_(0.0f),
          out_zr2_(0.0f),

          reachable_(true) {
        // 0除算を避ける。a,bは機構定数なので通常0にはならない
        if (std::abs(a_) < kEps) {
            a_ = kEps;
        }
        if (std::abs(b_) < kEps) {
            b_ = kEps;
        }
    }

    // 差動モータ自体の回転制限。r,zの可動域とは別に効かせたい場合に設定する
    void set_zr_limit(float zr_min, float zr_max) {
        zr_min_ = zr_min;
        zr_max_ = zr_max;
    }

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
        out_zr1_ = 0.0f;
        out_zr2_ = 0.0f;
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

        // 差動の分配: 和がz、差がrになるよう2つのモータ角へ振り分ける
        float zr1 = 0.5f * (out_z_ / a_ + out_r_ / b_);
        float zr2 = 0.5f * (out_z_ / a_ - out_r_ / b_);

        if (zr1 < zr_min_ || zr1 > zr_max_) {
            reachable_ = false;
        }
        if (zr2 < zr_min_ || zr2 > zr_max_) {
            reachable_ = false;
        }

        out_zr1_ = std::clamp(zr1, zr_min_, zr_max_);
        out_zr2_ = std::clamp(zr2, zr_min_, zr_max_);

        // モータ角をクランプするとr,zも変わるので、実際に到達する値へ戻しておく
        out_z_ = a_ * (out_zr1_ + out_zr2_);
        out_r_ = b_ * (out_zr1_ - out_zr2_);

        return reachable_;
    }

    // 実際にモータへ渡す指令値
    float get_zr1() const {
        return out_zr1_;
    }

    float get_zr2() const {
        return out_zr2_;
    }

    float get_theta() const {
        return out_theta_;
    }

    // 到達する手先位置。クランプが効いた場合は目標とずれる
    float get_r() const {
        return out_r_;
    }

    float get_z() const {
        return out_z_;
    }

    bool reachable() const {
        return reachable_;
    }

    // 順運動学: 差動モータ角から半径と高さを求める
    static void forward_differential(float a, float b, float zr1, float zr2, float &r, float &z) {
        z = a * (zr1 + zr2);
        r = b * (zr1 - zr2);
    }

    // 順運動学: 円筒座標から直交座標を求める。IKの結果を戻して往復テストする用
    static void forward(float r, float theta, float z, float &X, float &Y, float &Z) {
        X = r * std::cos(theta);
        Y = r * std::sin(theta);
        Z = z;
    }

private:
    static constexpr float kSingularRadius = 1e-3f;  // これ未満の半径はθ不定とみなす [m]
    static constexpr float kEps = 1e-9f;             // 0除算を避けるための下限

    float r_min_;
    float r_max_;
    float theta_min_;
    float theta_max_;
    float z_min_;
    float z_max_;

    float a_;       // z = a * (zr1 + zr2) の機構定数
    float b_;       // r = b * (zr1 - zr2) の機構定数
    float zr_min_;  // 差動モータの回転制限
    float zr_max_;

    float X_;  // 目標手先位置 [m]
    float Y_;  // 目標手先位置 [m]
    float Z_;  // 目標手先位置 [m]

    float out_r_;      // 到達する半径 [m]
    float out_theta_;  // 旋回角の指令値 [rad]
    float out_z_;      // 到達する高さ [m]
    float out_zr1_;    // 差動モータ1の指令値 [rad]
    float out_zr2_;    // 差動モータ2の指令値 [rad]

    bool reachable_;
};
