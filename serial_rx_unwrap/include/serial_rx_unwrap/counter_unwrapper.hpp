/*
ラップアラウンドする整数カウンタを連続値へ復元するユーティリティ。

serial_bridge/ros2can の各 int16 スロット(エンコーダカウンタ等)は±32768で
折り返すため、折り返し前後の生値の差分を監視し、半周期を超える跳躍をラップと
みなして補正することで連続値に復元する。

欠落量がちょうど半周期に近い場合、「絶対値最小」の判定だけでは原理的にどちらが
正しいか区別できない(実機検証で確認済み: ラップ境界付近で稀に大量のサンプルが
欠落する瞬間があり、絶対値最小ヒューリスティックが誤った側を選ぶことがある)。

これを緩和するため、差分の絶対値が半周期に近い「本当に曖昧な場合」に限り、
直近の移動方向(符号)と一致する候補を優先する。差分が明らかに小さい通常時
(静止中のノイズ程度の微小変動を含む)は、この判定を一切使わず素直に絶対値最小を
採用する(曖昧でない場合にまでトレンド判定を適用すると、静止中の符号反転する
微小ノイズを誤って1周期分の大ジャンプとして扱ってしまうため)。
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace serial_rx_unwrap {

class CounterUnwrapper {
public:
    explicit CounterUnwrapper(int64_t counts_per_wrap = 65536,
                               double ambiguous_margin_ratio = 0.1,
                               int64_t trend_noise_floor = 4)
        : counts_per_wrap_(counts_per_wrap),
          half_wrap_(static_cast<double>(counts_per_wrap) / 2.0),
          ambiguous_threshold_(half_wrap_ * (1.0 - ambiguous_margin_ratio)),
          trend_noise_floor_(trend_noise_floor) {}

    void reset() {
        prev_raw_.reset();
        unwrapped_ = 0;
        trend_sign_ = 0;
    }

    int64_t update(int64_t raw) {
        if (!prev_raw_.has_value()) {
            unwrapped_ = raw;
            prev_raw_ = raw;
            return unwrapped_;
        }

        const int64_t naive_delta = raw - prev_raw_.value();
        int64_t candidates[3] = {
            naive_delta,
            naive_delta - counts_per_wrap_,
            naive_delta + counts_per_wrap_,
        };
        std::sort(candidates, candidates + 3,
                  [](int64_t a, int64_t b) { return std::llabs(a) < std::llabs(b); });
        const int64_t best = candidates[0];
        const int64_t second = candidates[1];

        int64_t delta = best;
        if (std::llabs(best) >= ambiguous_threshold_ && trend_sign_ != 0) {
            const bool best_matches = (best > 0) == (trend_sign_ > 0);
            const bool second_matches = (second > 0) == (trend_sign_ > 0);
            if (!best_matches && second_matches) {
                delta = second;
            }
        }

        unwrapped_ += delta;
        prev_raw_ = raw;
        if (std::llabs(delta) >= trend_noise_floor_) {
            trend_sign_ = (delta > 0) ? 1 : -1;
        }
        return unwrapped_;
    }

private:
    int64_t counts_per_wrap_;
    double half_wrap_;
    double ambiguous_threshold_;
    int64_t trend_noise_floor_;

    std::optional<int64_t> prev_raw_;
    int64_t unwrapped_ = 0;
    int trend_sign_ = 0;  // -1, 0(未確定), +1
};

}  // namespace serial_rx_unwrap
