#pragma once
// serial_bridge / ros2can 互換の52バイト固定長フレーム実装。
// [0xAA][DEVICE_ID][LEN=48][int16 x24 big-endian][XOR checksum]
// チーム標準(RRST-NHK-Project/ros2can)のフレーム仕様に合わせてあるため、
// 将来 ros2can 側に MODE_DAMIAO として統合する際もそのまま流用できる。

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace damiao_ctrl
{

constexpr uint8_t kStartByte = 0xAA;
constexpr size_t kSlotCount = 24;
constexpr uint8_t kDataLength = kSlotCount * 2;  // 48
constexpr size_t kFrameLength = 1 + 1 + 1 + kDataLength + 1;  // 52

using SlotArray = std::array<int16_t, kSlotCount>;

// 24要素(不足分は0埋め、超過分は切り詰め)をビッグエンディアンの52バイトフレームへ変換する。
inline std::vector<uint8_t> encode_frame(uint8_t device_id, const SlotArray & data)
{
  std::vector<uint8_t> frame;
  frame.reserve(kFrameLength);

  uint8_t checksum = 0;
  frame.push_back(kStartByte);
  frame.push_back(device_id);
  frame.push_back(kDataLength);
  checksum ^= device_id;
  checksum ^= kDataLength;

  for (size_t i = 0; i < kSlotCount; ++i) {
    const uint16_t unsigned_v = static_cast<uint16_t>(data[i]);
    const uint8_t hi = static_cast<uint8_t>((unsigned_v >> 8) & 0xFF);
    const uint8_t lo = static_cast<uint8_t>(unsigned_v & 0xFF);
    frame.push_back(hi);
    frame.push_back(lo);
    checksum ^= hi;
    checksum ^= lo;
  }

  frame.push_back(checksum);
  return frame;
}

// ストリーミング受信バッファに対して1バイト単位で再同期するステートマシン。
// PC(ros2can frame_codec.py)・ESP32(serial_task.cpp)双方と対称的な実装。
class FrameParser
{
public:
  struct Frame
  {
    uint8_t device_id;
    SlotArray data;
  };

  void push_bytes(const uint8_t * buf, size_t len)
  {
    buffer_.insert(buffer_.end(), buf, buf + len);
  }

  // 有効な1フレームを取り出せた場合のみ値を返す。呼び出し側は戻り値が
  // std::nulloptになるまでループで呼び出すこと(1回の呼び出しで複数フレーム
  // が溜まっている場合があるため)。
  std::optional<Frame> pop_frame()
  {
    for (;;) {
      // 1) START_BYTEまで読み捨てる
      while (!buffer_.empty() && buffer_.front() != kStartByte) {
        buffer_.pop_front();
      }
      if (buffer_.size() < kFrameLength) {
        return std::nullopt;
      }

      // 2) LENGTHフィールドの妥当性チェック(異常なら1バイト捨てて再走査)
      const uint8_t length = buffer_[2];
      if (length != kDataLength) {
        buffer_.pop_front();
        continue;
      }

      // 3) チェックサム検証
      uint8_t checksum = 0;
      const uint8_t device_id = buffer_[1];
      checksum ^= device_id;
      checksum ^= length;
      SlotArray data{};
      for (size_t i = 0; i < kSlotCount; ++i) {
        const uint8_t hi = buffer_[3 + i * 2];
        const uint8_t lo = buffer_[3 + i * 2 + 1];
        checksum ^= hi;
        checksum ^= lo;
        const uint16_t unsigned_v = static_cast<uint16_t>((hi << 8) | lo);
        data[i] = static_cast<int16_t>(unsigned_v);
      }
      const uint8_t received_checksum = buffer_[kFrameLength - 1];

      if (checksum != received_checksum) {
        // ノイズ等でずれた可能性があるため1バイトだけ捨てて再同期する。
        buffer_.pop_front();
        continue;
      }

      // 有効フレームを確定し、消費した分だけバッファから取り除く。
      for (size_t i = 0; i < kFrameLength; ++i) {
        buffer_.pop_front();
      }
      return Frame{device_id, data};
    }
  }

private:
  std::deque<uint8_t> buffer_;
};

}  // namespace damiao_ctrl
