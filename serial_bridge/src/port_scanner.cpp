/*====================================================================
シリアルポートを走査しマイコンから送信されているIDを取得する。これにより接続順に関わらず各マイコンを識別できる。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "serial_bridge/port_scanner.hpp"

#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <iostream>
#include <map>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

//
// /dev/ttyACMx(Arduino, STM32) または /dev/ttyUSBx(ESP32) を列挙する
//
static std::vector<std::string> list_serial_ports() {
    std::vector<std::string> ports;

    // 探索パターンの定義
    std::vector<std::string> patterns = {
        "/dev/ttyUSB*",
        "/dev/ttyACM*"};

    // 各パターンで glob を実行
    // globってなに？？ >> ワイルドカード付きのパス検索を行う。コマンドで行うls /dev/ttyUSB*みたいなやつ。
    for (const auto &pattern : patterns) {
        std::cout << "[SCAN] pattern: " << pattern << std::endl;

        // 検索結果を格納する構造体
        glob_t g;

        // 初期化
        memset(&g, 0, sizeof(g));

        int ret = glob(pattern.c_str(), 0, NULL, &g);
        if (ret == 0) {
            for (size_t i = 0; i < g.gl_pathc; ++i) {
                std::cout << "[SCAN] found: " << g.gl_pathv[i] << std::endl;
                ports.emplace_back(g.gl_pathv[i]);
            }
        } else {
            std::cout << "[SCAN] none matched for pattern " << pattern << std::endl;
        }
        globfree(&g);
    }

    return ports;
}

//
// 指定ポートを開き、マイコンから ID を読む
// 成功 -> ID
// 失敗 -> -1
//
static int read_device_id(const std::string &port) {
    std::cout << "[OPEN] Trying port: " << port << std::endl;

    // ポートを開く、ノンブロッキングにしないのがポイント
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cout << "[OPEN] Failed to open " << port << std::endl;
        return -1;
    }

    std::cout << "[OPEN] Success opening " << port << " (fd=" << fd << ")" << std::endl;

    // シリアル設定
    termios tty{};
    tcgetattr(fd, &tty);
    // RAWモード、届いたバイトをそのまま読む
    cfmakeraw(&tty);

    // 入出力のボーレートの設定（115200bps）
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // ローカル制御＆受信有効（ほんとに必要なのかこれ？）
    tty.c_cflag |= (CLOCAL | CREAD);

    // readのブロッキング条件
    tty.c_cc[VMIN] = 1;  // 1バイト来るまで待つ
    tty.c_cc[VTIME] = 5; // 0.5秒

    // 設定の即時反映
    tcsetattr(fd, TCSANOW, &tty);
    std::cout << "[CONFIG] Serial configured for " << port << std::endl;

    // USB CDC リセット待ち
    std::cout << "[WAIT] Waiting 1000ms for device reset..." << std::endl;
    usleep(1000000); // 1000ms

    // ID 読み取り
    // 複数回のreadを試みる（試行回数は減らす予定）
    unsigned char buf[1];
    std::cout << "[READ] Waiting up to 2s for ID on " << port << std::endl;

    for (int i = 0; i < 5; i++) {
        int n = read(fd, buf, 1);
        if (n == 1) {
            std::cout << "[READ] Got ID=0x" << std::hex << (int)buf[0]
                      << " (" << std::dec << (int)buf[0] << ")" << std::endl;

            // ACK送信（通信が確率したことをマイコンに知らせる）
            uint8_t ack = 0x20;
            write(fd, &ack, 1);

            std::cout << "[WRITE] Sent ACK to " << port << std::endl;
            close(fd);
            return buf[0];
        }
        usleep(100000); // 100ms
    }

    std::cout << "[READ] Timeout: No ID received from " << port << std::endl;

    close(fd);
    return -1;
}

//
// 全ポートを走査して ID → ポート名 の map を返す
//
std::map<uint8_t, std::string> detect_serial_devices() {
    std::map<uint8_t, std::string> result;

    auto ports = list_serial_ports();

    std::cout << "[SCAN] Total ports found: " << ports.size() << std::endl;

    for (auto &p : ports) {
        std::cout << "[CHECK] Checking port: " << p << std::endl;

        int id = read_device_id(p);
        if (id >= 0) {
            result[(uint8_t)id] = p;
            std::cout << "[OK] Detected ID=" << id << " on " << p << std::endl;
        } else {
            std::cout << "[NG] No valid ID from " << p << std::endl;
        }
    }

    if (result.empty()) {
        std::cout << "[RESULT] No serial devices detected." << std::endl;
    } else {
        std::cout << "[RESULT] Total detected devices: " << result.size() << std::endl;
    }

    return result;
}
