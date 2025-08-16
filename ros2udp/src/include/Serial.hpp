// Serial.hpp
// 自動識別機能付きシリアル通信クラス開発中
#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Serial {
public:
    Serial(const std::string &device_path, int baudrate = 115200);
    ~Serial();

    void send(const std::vector<int16_t> &data);

private:
    int serial_fd;
};
