#include "encoder_angle/counter_unwrapper.hpp"

#include <gtest/gtest.h>

using encoder_angle::CounterUnwrapper;

namespace {
// 2の補数オーバーフロー(全域リング)を模したヘルパー。汎用ユーティリティとしての
// CounterUnwrapper 自体は counts_per_wrap を汎用パラメータとして受け取れることを
// 検証するために使う(実機のPCNTはこのモデルではない。下記の
// PcntResetToZero系のテストが実機の実際の挙動に対応する)。
int64_t to_int16(int64_t x) {
    x = ((x % 65536) + 65536) % 65536;
    if (x >= 32768) {
        x -= 65536;
    }
    return x;
}

// xiao-esp32-s3_can2io の実機PCNTを模したヘルパー。
// 生カウントは +32767到達で0へ、-32768到達で0へ、それぞれ独立にリセットされる
// (実機観測で確認済み。counter_unwrapper.hpp 冒頭コメント参照)。
// unbounded な真の物理位置 true_val から、実機が返すであろう生値(int16)を求める。
int64_t to_pcnt_reset(int64_t true_val) {
    int64_t r = 0;
    if (true_val >= 0) {
        r = true_val % 32768;  // 0..32767 を繰り返す
    } else {
        r = -((-true_val) % 32768);  // 0..-32767(実質) を繰り返す
    }
    return r;
}
}  // namespace

TEST(CounterUnwrapper, DenseStreamTracksTrueValueExactly) {
    // 汎用ユーティリティとして、任意の counts_per_wrap (ここでは65536)でも
    // 正しく機能することの確認。
    CounterUnwrapper u(65536);
    int64_t true_val = 0;
    for (int step = 0; step < 2000; ++step) {
        true_val += 50;
        const int64_t raw = to_int16(true_val);
        EXPECT_EQ(u.update(raw), true_val) << "step=" << step;
    }
}

TEST(CounterUnwrapper, RealWorldLargeGapAtWrapBoundaryResolvesInTrendDirection) {
    // 実機で観測された「半周期にわずかに満たない欠落」のケース。
    // 直近のトレンドは負方向(-84/step程度)で確立されている。
    CounterUnwrapper u(65536);
    const int64_t seq[] = {
        -32052, -32139, -32224, -32308, -32392, -32475, -32561, -32646, -32716, -33,
    };
    int64_t last = 0;
    int64_t prev = 0;
    for (int64_t raw : seq) {
        prev = last;
        last = u.update(raw);
    }
    EXPECT_LT(last - prev, 0) << "半周期未満の大欠落はトレンド方向(負)に解決されるべき";
}

TEST(CounterUnwrapper, IdleNoiseDoesNotCauseLargeJump) {
    // 直近のトレンド(負方向)を確立させた後、静止中の符号反転する微小ノイズを与える。
    CounterUnwrapper u(65536);
    for (int64_t raw : {-100, -110, -120, -130}) {
        u.update(raw);
    }
    const int64_t noisy[] = {-131, -130, -132, -131, -129, -131, -130, -131};
    int64_t prev = -130;
    int64_t max_jump = 0;
    for (int64_t raw : noisy) {
        const int64_t result = u.update(raw);
        const int64_t jump = std::abs(result - prev);
        max_jump = (jump > max_jump) ? jump : max_jump;
        prev = result;
    }
    EXPECT_LT(max_jump, 10) << "静止中のノイズで1周期分の大ジャンプが発生してはいけない";
}

TEST(CounterUnwrapper, ResetClearsState) {
    CounterUnwrapper u(65536);
    u.update(32000);
    u.update(-32000);
    u.reset();
    EXPECT_EQ(u.update(5), 5);
}

// --- ここから下は xiao-esp32-s3_can2io の実機PCNT挙動 (counts_per_wrap=32768) ---

TEST(CounterUnwrapper, PcntResetToZeroPositiveDirectionDenseStream) {
    // +方向へ回転させ、32767到達で0へリセットされるケース。
    CounterUnwrapper u(32768);
    int64_t true_val = 0;
    for (int step = 0; step < 2000; ++step) {
        true_val += 37;
        const int64_t raw = to_pcnt_reset(true_val);
        EXPECT_EQ(u.update(raw), true_val) << "step=" << step;
    }
}

TEST(CounterUnwrapper, PcntResetToZeroNegativeDirectionDenseStream) {
    // -方向へ回転させ、-32768到達で0へリセットされるケース
    // (ユーザーが実機で確認した "-32767 -> 0" のシナリオに対応)。
    CounterUnwrapper u(32768);
    int64_t true_val = 0;
    for (int step = 0; step < 2000; ++step) {
        true_val -= 41;
        const int64_t raw = to_pcnt_reset(true_val);
        EXPECT_EQ(u.update(raw), true_val) << "step=" << step;
    }
}

TEST(CounterUnwrapper, PcntResetToZeroMultipleWrapsBothDirections) {
    // 正負を行き来しながら複数回リセットをまたぐケース。
    // (各ステップで真値と一致することをその場で検証するのが目的なので、
    // 最終的な累積値そのものには意味を持たせていない)
    CounterUnwrapper u(32768);
    int64_t true_val = 0;
    const int64_t steps[] = {500, 500, 500, -300, -300, -300, -300, 900, 900, -700, -700, -700};
    for (int64_t step : steps) {
        for (int i = 0; i < 40; ++i) {
            true_val += step;
            const int64_t raw = to_pcnt_reset(true_val);
            ASSERT_EQ(u.update(raw), true_val);
        }
    }
}
