#pragma once

#include <iostream>
#include <string>
#include "Constants.h" // FieldData::getBlockNum のため

namespace Log {
    // 通常のメッセージ
    inline void info(const std::string& msg) {
        std::cout << msg << std::endl;
    }

    // 位置情報付きメッセージ
    inline void pos(const std::string& msg, Position p) {
         std::cout << msg << " (ブロック " 
                   << FieldData::getBlockNum(p) << ")" << std::endl;
    }

    // [R2] などのプレフィックス付き
    inline void R2(const std::string& msg) {
        std::cout << "[R2] " << msg << std::endl;
    }

    // [スキャン] などのプレフィックス付き
    inline void Scan(const std::string& msg) {
        std::cout << "  [スキャン] " << msg << std::endl;
    }

    // [回収] などのプレフィックス付き
    inline void Pickup(const std::string& msg) {
        std::cout << "  [回収] " << msg << std::endl;
    }
} // namespace Log