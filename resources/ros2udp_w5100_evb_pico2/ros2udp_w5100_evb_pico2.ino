/*
RRST-NHK-Project-2025
 */


#include <Ethernet.h>
#include <EthernetUdp.h>
#include <vector>          // std::vector

// MACアドレスと静的IPアドレスの設定
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // 任意のMACアドレス、変更いるかも？
IPAddress ip(192, 168, 0, 217);                    // 静的IPアドレス
IPAddress gateway(192, 168, 0, 1);                 // デフォルトゲートウェイ
IPAddress subnet(255, 255, 255, 0);                // サブネットマスク
unsigned int localPort = 5000; // ポート

// UDPのインスタンス化
EthernetUDP udp;

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH Shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet

  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
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

