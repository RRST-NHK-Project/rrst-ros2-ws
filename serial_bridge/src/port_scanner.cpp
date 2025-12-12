#include "serial_bridge/port_scanner.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

//
// /dev/ttyACMx または /dev/ttyUSBx を列挙する
//
static std::vector<std::string> list_serial_ports() {
    std::vector<std::string> result;

    const char *dev_dir = "/dev";
    DIR *dir = opendir(dev_dir);
    if (!dir)
        return result;

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name(entry->d_name);

        if (name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0) {
            result.push_back("/dev/" + name);
        }
    }
    closedir(dir);
    return result;
}

//
// 指定ポートを開き、マイコンから ID を読む
// 成功 → ID
// 失敗 → -1
//
static int read_device_id(const std::string &port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;

    // シリアル設定
    termios tty{};
    tcgetattr(fd, &tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &tty);

    usleep(200000); // 200ms 待つ（マイコン初回送信を待つ）

    unsigned char buf[1];
    int n = read(fd, buf, 1);
    close(fd);

    if (n == 1) {
        return buf[0]; // 0x00〜
    }
    return -1;
}

//
// 全ポートを走査して ID → ポート名 の map を返す
//
std::map<uint8_t, std::string> detect_serial_devices() {
    std::map<uint8_t, std::string> result;

    auto ports = list_serial_ports();
    for (auto &p : ports) {
        int id = read_device_id(p);
        if (id >= 0) {
            result[(uint8_t)id] = p;
            std::cout << "Detected ID=" << id << " on " << p << std::endl;
        }
    }
    return result;
}
