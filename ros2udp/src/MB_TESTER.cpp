/*
RRST-NHK-Project 2025
メイン基板ののテスト用
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#define MC_PRINTF 1 // マイコン側のprintfを無効化・有効化(0 or 1)

// IPアドレスとポートの指定
std::string udp_ip = "192.168.0.216"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

std::vector<int16_t> data(19, 0); // 7~9番を電磁弁制御に転用中（-1 or 1）
/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | Servo1 | 0 ~ 270 |
| data[8] | Servo2 | 0 ~ 270 |
| data[9] | Servo3 | 0 ~ 270 |
| data[10] | Servo4 | 0 ~ 270 |
| data[11] | TR1 | 0 or 1|
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|
| data[14] | TR4 | 0 or 1|
| data[15] | TR5 | 0 or 1|
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|
*/
class KeyboardInputHandler {
public:
    KeyboardInputHandler(UDP &udp) : udp_(udp), running_(true) {
        input_thread_ = std::thread(&KeyboardInputHandler::keyboard_input_loop, this);
    }

    ~KeyboardInputHandler() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void keyboard_input_loop() {
        while (running_) {
            int index, value;
            std::cout << "Enter index (0-18) and value: ";

            if (!(std::cin >> index >> value)) {
                std::cerr << "Invalid input! Please enter two integers." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            if (index < 0 || index >= 19) {
                std::cerr << "Invalid index! Enter a value between 0 and 18." << std::endl;
                continue;
            }

            // データ範囲のチェック
            if (index >= 1 && index <= 6) { // MD1~MD6
                value = std::clamp(value, -100, 100);
            } else if (index >= 7 && index <= 10) { // サーボ
                value = std::clamp(value, 0, 270);
            } else if (index >= 11 && index <= 18) { // TR
                value = (value != 0) ? 1 : 0;
            }

            data[index] = value;
            data[0] = MC_PRINTF; // マイコン側のprintfを無効化・有効化(0 or 1)
            std::cout << "Updated data[" << index << "] = " << value << std::endl;

            // 変更後のdataを表示
            print_data();

            udp_.send(data);
        }
    }

    void print_data() {
        std::cout << "Current data: [";
        for (size_t i = 0; i < data.size(); ++i) {
            std::cout << data[i];
            if (i < data.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    UDP &udp_;
    std::thread input_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet MB TESTER";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cerr << "Please install 'figlet' with the following command:" << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }
    std::cout << "Keyboard input mode enabled. Enter index and value." << std::endl;

    // UDPオブジェクトを作成
    UDP udp(udp_ip, udp_port);

    // キーボード入力ハンドラーを開始
    KeyboardInputHandler keyboard_handler(udp);

    // 無限ループで終了を防ぐ
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
