// 実機なしでフレームコーデックの往復変換とノイズ耐性を確認するツール。
// `ros2 run damiao_ctrl frame_codec_selftest` で実行する。
#include <cstdio>
#include <random>

#include "damiao_ctrl/frame_codec.hpp"

using namespace damiao_ctrl;

namespace
{
bool check(bool cond, const char * name)
{
  std::printf("[%s] %s\n", cond ? "PASS" : "FAIL", name);
  return cond;
}
}  // namespace

int main()
{
  bool all_ok = true;

  // 1) 基本的な往復変換(正負混在の値)
  {
    SlotArray src{};
    for (size_t i = 0; i < kSlotCount; ++i) {
      src[i] = static_cast<int16_t>((i % 2 == 0) ? (100 * static_cast<int>(i)) : (-100 * static_cast<int>(i)));
    }
    const auto encoded = encode_frame(7, src);
    all_ok &= check(encoded.size() == kFrameLength, "encoded size == 52 bytes");

    FrameParser parser;
    parser.push_bytes(encoded.data(), encoded.size());
    auto frame = parser.pop_frame();
    all_ok &= check(frame.has_value(), "decode succeeds");
    if (frame) {
      all_ok &= check(frame->device_id == 7, "device_id round-trip");
      all_ok &= check(frame->data == src, "slot data round-trip");
    }
  }

  // 2) 先頭にゴミバイトが混入していても再同期できること
  {
    SlotArray src{};
    src[0] = -1234;
    const auto encoded = encode_frame(1, src);
    std::vector<uint8_t> noisy{0x00, 0xFF, 0xAA /* 偽のSTART */};
    noisy.insert(noisy.end(), encoded.begin(), encoded.end());

    FrameParser parser;
    parser.push_bytes(noisy.data(), noisy.size());
    auto frame = parser.pop_frame();
    all_ok &= check(frame.has_value() && frame->device_id == 1, "resync after garbage bytes");
  }

  // 3) チェックサム破損フレームは棄却され、後続の正常フレームは復元できること
  {
    SlotArray src{};
    src[5] = 42;
    auto encoded = encode_frame(2, src);
    std::vector<uint8_t> stream = encoded;  // 1本目: 破損させる
    stream.back() ^= 0xFF;  // checksumを壊す
    const auto good = encode_frame(3, src);
    stream.insert(stream.end(), good.begin(), good.end());  // 2本目: 正常

    FrameParser parser;
    parser.push_bytes(stream.data(), stream.size());
    auto frame = parser.pop_frame();
    all_ok &= check(frame.has_value() && frame->device_id == 3, "corrupted frame rejected, next frame recovered");
  }

  std::printf(all_ok ? "\nRESULT: ALL TESTS PASSED\n" : "\nRESULT: SOME TESTS FAILED\n");
  return all_ok ? 0 : 1;
}
