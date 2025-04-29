// Serial.cpp
// 自動識別機能付きシリアル通信クラス開発中
#include "Serial.hpp"

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

Serial::Serial(const std::string &device_path, int baudrate) {
    serial_fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        throw std::runtime_error("Failed to open serial device: " + device_path);
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        throw std::runtime_error("Error from tcgetattr");
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error from tcsetattr");
    }
}

Serial::~Serial() {
    if (serial_fd >= 0) {
        close(serial_fd);
    }
}

void Serial::send(const std::vector<int16_t> &data) {
    std::vector<uint8_t> packet;

    // ヘッダー
    packet.push_back(0xAA);
    packet.push_back(0x55);

    // サイズ（最大32個）
    uint8_t size = static_cast<uint8_t>(std::min(data.size(), static_cast<size_t>(32)));
    packet.push_back(size);

    // データ（リトルエンディアン）
    for (size_t i = 0; i < size; ++i) {
        int16_t val = data[i];
        packet.push_back(val & 0xFF);
        packet.push_back((val >> 8) & 0xFF);
    }

    ssize_t written = write(serial_fd, packet.data(), packet.size());
    if (written < 0) {
        std::cerr << "Serial write failed: " << strerror(errno) << std::endl;
    }
}
