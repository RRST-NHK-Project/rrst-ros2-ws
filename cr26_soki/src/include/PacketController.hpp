// マイコンへ送信する配列data_へ安全にアクセスするためのライブラリ

// ===== 使用例コメント =====
/*
// 変数宣言（PacketController のインスタンス）
PacketController ctrlPkt;

// debugフラグを立てる
ctrlPkt.setDebug(true);

// MD3を-80に設定
ctrlPkt.setMD(MD3, -80);

// SERVO2を90度に設定
ctrlPkt.setServo(SERVO2, 90);

// TR5をONにする
ctrlPkt.setTR(TR5, true);

// 直接data_にアクセスしてMD1を書き換え
ctrlPkt.data_[MD1] = 50;

// operator[]を使ってSERVO1を書き換え
ctrlPkt[SERVO1] = 120;

// 送信用配列を取得
auto sendArray = ctrlPkt.toArray();
*/

#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

// 配列インデックス enum
enum DataIndex : int {
    DEBUG = 0,
    MD1,
    MD2,
    MD3,
    MD4,
    MD5,
    MD6,
    MD7,
    MD8, // 1~8
    SERVO1,
    SERVO2,
    SERVO3,
    SERVO4,
    SERVO5,
    SERVO6,
    SERVO7,
    SERVO8, // 9~16
    TR1,
    TR2,
    TR3,
    TR4,
    TR5,
    TR6,
    TR7 // 17~23
};

constexpr int DATA_SIZE = 24;

class PacketController {
public:
    // 配列本体
    std::array<int16_t, DATA_SIZE> data_{};

    // 制限付きのアクセス関数
    void setDebug(bool value) { data_[DEBUG] = value ? 1 : 0; }

    void setMD(int index, int value) {
        if (index < 1 || index > 8)
            return;
        data_[index] = std::clamp(value, -255, 255);
    }

    void setServo(int index, int value) {
        if (index < 9 || index > 16)
            return;
        data_[index] = std::clamp(value, 0, 270);
    }

    void setTR(int index, bool value) {
        if (index < 17 || index > 23)
            return;
        data_[index] = value ? 1 : 0;
    }

    // 配列を vector で返す
    std::vector<int16_t> toVector() const {
        return std::vector<int16_t>(data_.begin(), data_.end());
    }

    // 直接アクセスも残す
    int16_t &operator[](int index) { return data_[index]; }
    const int16_t &operator[](int index) const { return data_[index]; }
};