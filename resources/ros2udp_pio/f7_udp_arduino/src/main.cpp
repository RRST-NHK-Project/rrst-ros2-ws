#include <EthernetUdp.h>   // UDP通信
#include <STM32Ethernet.h> // STM32用Ethernet
#include <vector>          // std::vector

// MACアドレスと静的IPアドレスの設定
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // 任意のMACアドレス、変更いるかも？
IPAddress ip(192, 168, 0, 217);                    // 静的IPアドレス
IPAddress gateway(192, 168, 0, 1);                 // ゲートウェイのIPアドレス（通常はルーター）
IPAddress subnet(255, 255, 255, 0);                // サブネットマスク

unsigned int localPort = 5000; // 受信するポート番号

// UDPのインスタンス作成
EthernetUDP udp;

void setup() {
    // シリアルモニタの初期化
    Serial.begin(9600);
    while (!Serial) {
        ;
    } // シリアル接続が確立するのを待つ

    // Ethernetの初期化（静的IP設定）
    Ethernet.begin(mac, ip);

    // IPアドレスが0.0.0.0であれば、Ethernet初期化が失敗したと判断
    if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
        Serial.println("Ethernet initialization failed.");
        while (true)
            ; // 無限ループで停止
    }

    // Ethernet設定を表示
    Serial.println("Ethernet initialized");
    Serial.print("IP Address: ");
    Serial.println(Ethernet.localIP());

    // UDPの初期化とポートのバインド
    udp.begin(localPort);
    Serial.print("Listening on port ");
    Serial.println(localPort);
}

void loop() {
    int packetSize = udp.parsePacket(); // 受信したパケットのサイズを取得
    if (packetSize) {
        // 受信したパケットのサイズと送信元情報を表示
        // Serial.print("Received packet of size: ");
        // Serial.println(packetSize);
        // IPAddress remoteIP = udp.remoteIP();  // 送信元IPアドレスを取得
        // Serial.print("From IP: ");
        // Serial.println(remoteIP);

        // std::vector<int16_t> に受信データを格納 (19個のサイズに固定)
        std::vector<int16_t> data(19, 0);                                 // 19個の要素を0で初期化
        int len = udp.read((uint8_t *)data.data(), 19 * sizeof(int16_t)); // 19個分のデータを受信

        if (len > 0) {
            // データ受信後の処理はここ！
            if (data[0] == 1 || data[0] == -1) {
                // 受信データをカンマ区切りでシリアルモニタに表示
                for (size_t i = 0; i < data.size(); i++) {
                    Serial.print(data[i]);

                    // 最後の要素以外はカンマを表示
                    if (i < data.size() - 1) {
                        Serial.print(", ");
                    }
                }
                Serial.println(); // 最後に改行を追加
            }
        } else {
            Serial.println("Error in receiving data.");
        }
    } else {
        // Serial.println("No packet received.");
    }

    // delay(10);  // 少し待機
}
