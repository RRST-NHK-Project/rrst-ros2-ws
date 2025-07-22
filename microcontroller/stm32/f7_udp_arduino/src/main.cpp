#include <EthernetUdp.h>   // UDP通信
#include <STM32Ethernet.h> // STM32用Ethernet
#include <vector>          // std::vector

// ピンの定義 //
//  MD PWM
#define MD1P PA_0
#define MD2P PA_3
#define MD3P PC_7
#define MD4P PC_6
#define MD5P PC_8
#define MD6P PC_9

// MD DIR
#define MD1D PD_2
#define MD2D PG_2
#define MD3D PD_5
#define MD4D PD_6
#define MD5D PD_7
#define MD6D PC_10

// サーボ
#define SERVO1 PB_1
#define SERVO2 PB_6
#define SERVO3 PD_13
#define SERVO4 PB_8

// トランジスタ（ソレノイド・表示灯）
#define TR1 PF_0
#define TR2 PF_1
#define TR3 PF_15
#define TR4 PC_11
#define TR5 PC_12
#define TR6 PF_14
#define TR7 PF_12
#define TR8 PF_13

// MACアドレスと静的IPアドレスの設定
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // 任意のMACアドレス、変更いるかも？
IPAddress ip(192, 168, 0, 217);                    // 静的IPアドレス
IPAddress gateway(192, 168, 0, 1);                 // デフォルトゲートウェイ
IPAddress subnet(255, 255, 255, 0);                // サブネットマスク

unsigned int localPort = 5000; // ポート

// UDPのインスタンス化
EthernetUDP udp;

void setup() {
    Serial.begin(9600); // デバッグ用のシリアル通信
    while (!Serial) {
        ;
    } // 確立するまで待機

    // Ethernetの初期化（静的IP設定）
    Ethernet.begin(mac, ip);

    // IPアドレスが0.0.0.0であれば、Ethernet初期化が失敗したと判断
    if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
        Serial.println("Ethernet initialization failed.");
        while (true)
            ; // エラーハンドリング、追加の処理実装予定
    }

    // Ethernet設定を表示
    Serial.println("Ethernet initialized");
    Serial.print("IP Address: ");
    Serial.println(Ethernet.localIP());

    // UDPの初期化とポートのバインド
    udp.begin(localPort);
    Serial.print("Listening on port ");
    Serial.println(localPort);

    // ピンの初期化 //
    // MD DIR
    pinMode(MD1P, OUTPUT);
    pinMode(MD2P, OUTPUT);
    pinMode(MD3P, OUTPUT);
    pinMode(MD4P, OUTPUT);
    pinMode(MD5P, OUTPUT);
    pinMode(MD6P, OUTPUT);

    // MD PWM
    pinMode(MD1D, OUTPUT);
    pinMode(MD2D, OUTPUT);
    pinMode(MD3D, OUTPUT);
    pinMode(MD4D, OUTPUT);
    pinMode(MD5D, OUTPUT);
    pinMode(MD6D, OUTPUT);

    // サーボ
    pinMode(SERVO1, OUTPUT);
    pinMode(SERVO2, OUTPUT);
    pinMode(SERVO3, OUTPUT);
    pinMode(SERVO4, OUTPUT);

    // トランジスタ(ソレノイド・表示灯)
    pinMode(TR1, OUTPUT);
    pinMode(TR2, OUTPUT);
    pinMode(TR3, OUTPUT);
    pinMode(TR4, OUTPUT);
    pinMode(TR5, OUTPUT);
    pinMode(TR6, OUTPUT);
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

        std::vector<int16_t> data(19, 0);                                 // 受信データを格納する配列data
        int len = udp.read((uint8_t *)data.data(), 19 * sizeof(int16_t)); // 受信データをdataに格納
        /*
        受信するデータの内容
        このデータに合わせてIOを操作する
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

        if (len > 0) {
            // データ受信後の処理はここ！//

            // デバッグ用（デバッグモード、緊急停止中のみprint）//
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

            // 以降IOの操作 //

            // MD
            // ソレノイド
            // サーボ
            // トランジスタ

        } else {
            Serial.println("Error in receiving data.");
        }
    } else {
        // Serial.println("No packet received.");
    }

    // delay(10);  // 少し待機
}
