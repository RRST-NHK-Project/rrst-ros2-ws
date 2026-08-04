#include "serial_rx_unwrap/counter_unwrapper.hpp"

#include <gtest/gtest.h>

using serial_rx_unwrap::CounterUnwrapper;

namespace {
int64_t to_int16(int64_t x) {
    x = ((x % 65536) + 65536) % 65536;
    if (x >= 32768) {
        x -= 65536;
    }
    return x;
}
}  // namespace

TEST(CounterUnwrapper, DenseStreamTracksTrueValueExactly) {
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
    bool first = true;
    for (int64_t raw : seq) {
        prev = last;
        last = u.update(raw);
        (void)first;
        first = false;
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
