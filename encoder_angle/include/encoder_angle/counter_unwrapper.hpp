/*
ラップアラウンドする int16 カウンタを連続値へ復元するユーティリティ。

[ESP32 PCNT の仕様確認結果 (実機で確認済み)]
xiao-esp32-s3_can2io (ros2can/firmware/xiao-esp32-s3_can2io) の
pin_ctrl_init.cpp は pcnt_config_t::counter_h_lim/counter_l_lim に
int16_t の最大/最小値(32767 / -32768)を設定して pcnt_unit_config() を
呼んでいる。ESP-IDF の driver/pcnt.h は pcnt_unit_config() の doxygen
コメントに「この関数は PCNT_EVT_L_LIM/H_LIM/ZERO の3イベントを無効化する」
と書かれているため、当初は「イベント無効 = 上限/下限到達時の自動0クリアも
無効になり、生値は素の16bit符号あり(2の補数)として+32767/-32768間で単純に
オーバーフローする」と推測したが、これは誤りだった。実機でエンコーダーを
負方向へ回転させて ros2can の serial_rx_[ID] を直接観測したところ、
生値は -32767 に達すると(2の補体の -32768 を経由せず)そのまま 0 へ戻り、
0 から再び負方向へカウントが続くことを確認した。すなわち
「イベント(割り込み通知)の無効化」と「h_lim/l_lim到達時の0クリア動作」は
別物であり、後者はイベント有効/無効に関わらずハードウェアの比較器
(cnt_h_lim_un/cnt_l_lim_un レジスタ、常時ロードされている)によって
常時作動している。

この結果、生値は「+側は32767に達すると0へ、-側は-32768に達すると
(観測上は-32767の次に)0へ、それぞれ独立にリセットされる」という挙動になる。
h_lim=32767, l_lim=-32768 という設定から、このリセット幅は正負どちらの方向も
32768 (= h_lim+1 = -l_lim) で対称になる。これは単純な2の補数オーバーフロー
(全域65536幅の1本のリング)とは異なり、0を境に正負それぞれ独立に
0までリセットされる構造だが、幸い正負どちらのリセットも補正量の絶対値は
counts_per_wrap=32768 で共通(符号だけがリセット方向に応じて変わる)なので、
本クラスの「候補{差分, 差分-32768, 差分+32768}のうち絶対値最小を選ぶ」
というロジックがそのまま正しく機能する(候補生成に使う値を65536ではなく
32768にするだけでよい)。

「絶対値最小」ヒューリスティック(生値の差分を求め、+counts_per_wrap/
-counts_per_wrap した候補と比べて絶対値が最小のものを真の差分とみなす)は
密なサンプルでは正しく機能するが、実機検証で次の問題が発覚した: 欠落量
(サンプル間隔)がちょうど半周期(counts_per_wrap/2)に近いと、正負どちらの
候補も絶対値がほぼ同じになり、「絶対値最小」だけでは原理的に正しい側を
選べない場合がある。これを緩和するため、差分の絶対値が半周期に近い
「本当に曖昧な場合」に限り、直近の移動方向(符号)と一致する候補を優先する。
差分が明らかに小さい通常時(静止中のノイズ程度の微小変動を含む)は、この判定を
一切使わず素直に絶対値最小を採用する(曖昧でない場合にまでトレンド判定を
適用すると、静止中の符号反転する微小ノイズを誤って1周期分の大ジャンプとして
扱ってしまうため)。
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace encoder_angle {

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

}  // namespace encoder_angle
