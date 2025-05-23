/*
RRST NHK2025
IPアドレスは適宜変更
FB制御に対応
UDPでエンコーダーの角度を送信しROS側でPID制御する
メイン基板V1.3
2025/05/12
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>
#include <vector>

#define PI 3.141592653589793

//---------------------------QEI---------------------------//
QEI ENC1(PC_0, PG_1, NC, 48, QEI::X2_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 48, QEI::X2_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 48, QEI::X2_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 48, QEI::X2_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる
データシート: https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/

void receive(UDPSocket *receiver);

// マッピング関数
int map(int value, int inMin, int inMax, int outMin, int outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// PWM
PwmOut MD1P(PA_0);
PwmOut MD2P(PA_3);
PwmOut MD3P(PC_7);
PwmOut MD4P(PC_6);
PwmOut MD5P(PC_8);
PwmOut MD6P(PC_9);

// DIR
DigitalOut MD1D(PD_2);
DigitalOut MD2D(PG_2);
DigitalOut MD3D(PD_5);
DigitalOut MD4D(PD_6);
DigitalOut MD5D(PD_7);
DigitalOut MD6D(PC_10);

//サーボ
PwmOut SERVO1(PB_1);
PwmOut SERVO2(PB_6);
PwmOut SERVO3(PD_13);
PwmOut SERVO4(PB_8);

//トランジスタ（電磁弁・表示灯用）
DigitalOut TR1(PF_0);
DigitalOut TR2(PF_1);
DigitalOut TR3(PF_15);
DigitalOut TR4(PC_11);
DigitalOut TR5(PC_12);
DigitalOut TR6(PF_14);
DigitalOut TR7(PF_12);
DigitalOut TR8(PF_13);

// CAN
CAN can{PD_0, PD_1, (int)1e6}; // rd,td,1Mhz

// グローバル変数の定義
int Pulse[4];    // エンコーダーのパルス格納用
int PPRx2 = 96; // エンコーダーのResolution
float Deg[4];

// グローバル変数の定義

double mdd[7]; // MDに出力する方向指令を格納
double mdp[7]; // MDに出力するduty比を格納

const char *recievefromIP = nullptr; //ネットワーク切断検知用

int main() {

  // PWM周波数の設定
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  MD6P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz) * 10^3
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */

  //サーボのPWM周波数の設定
  SERVO1.period_ms(20);
  SERVO2.period_ms(20);
  SERVO3.period_ms(20);
  SERVO4.period_ms(20);

  // 自機情報
  // const char *myIP = "192.168.8.215"; // MR_SD
  // const char *myIP = "192.168.8.216"; // MR
  const char *myIP = "192.168.0.217"; // DR_SD
  // const char *myIP = "192.168.0.218"; // DR
  // const char *myIP = "192.168.128.215"; // DR on test
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;


  // 送信先情報
  const char *destinationIP = "192.168.0.100";
  const uint16_t destinationPort = 4000;

  //クラスのインスタンス化
  EthernetInterface net;
  SocketAddress destination, source, myData;
  UDPSocket udp;
  Thread receiveThread;

  //切断検知関連
  SocketAddress test_dest;
  test_dest.set_ip_address(
      "192.168.0.1"); //接続するルーターのゲートウェイを指定する
  test_dest.set_port(5001);
  char test_data[] = "ping";
  bool is_disconnected = false;
  //ここまで

  // DHCPオフ（IPは固定）
  net.set_dhcp(false);
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("[警告] ネットワーク接続に失敗\n");
    return -1;
  } else {
    printf("[情報] ネットワーク接続に成功\n");
    printf("IP: %s\n", myIP);
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

   // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);

  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  // メインループ（送信用）
  while (1) {

    nsapi_size_or_error_t result =
        udp.sendto(test_dest, test_data, sizeof(test_data));
    if (result < 0) {
      if (!is_disconnected) {
        printf("[警告] ネットワークから切断されました\n");
        printf("[警告] 緊急停止！\n");
        MD1P = 0;
        MD2P = 0;
        MD3P = 0;
        MD4P = 0;
        MD5P = 0;
        MD6P = 0;
        is_disconnected = true;
      }
    } else {
      if (is_disconnected) {
        printf("[情報] ネットワークに復帰しました\n");
        is_disconnected = false;
      }
    }

    // エンコーダーの値を取得
    Pulse[0] = ENC1.getPulses();
    Pulse[1] = ENC2.getPulses();
    Pulse[2] = ENC3.getPulses();
    Pulse[3] = ENC4.getPulses();

    //エンコーダーのパルスを角度に変換
    Deg[0] = 360.0 / (float)PPRx2 * (float)Pulse[0];
    Deg[1] = 360.0 / (float)PPRx2 * (float)Pulse[1];
    Deg[2] = 360.0 / (float)PPRx2 * (float)Pulse[2];
    Deg[3] = 360.0 / (float)PPRx2 * (float)Pulse[3];

        // UDP送信処理
    float send_data[4] = {Deg[0], Deg[1], Deg[2], Deg[3]}; // Deg[]の送信
    nsapi_size_or_error_t result_enc = udp.sendto(destination, send_data, sizeof(send_data));
    if (result_enc < 0) {
        printf("[送信エラー] UDP送信に失敗しました: %d\n", result);
    } else {
        //printf("送信した角度: %.2f, %.2f, %.2f, %.2f\n", send_data[0], send_data[1], send_data[2], send_data[3]);
    }

    ThisThread::sleep_for(10ms);
  }

  // スレッドの終了を待つ
  receiveThread.join();

  // UDPソケットを閉じ、ネットワーク接続を切断
  udp.close();
  net.disconnect();
  return 0;
}

void receive(UDPSocket *receiver) { // UDP受信スレッド

  using namespace std::chrono;

  SocketAddress source;
  std::vector<int16_t> data(19, 0); // 19要素の整数ベクトルを0で初期化

  while (1) {

    int recv_size =
        receiver->recvfrom(&source, reinterpret_cast<char *>(data.data()),
                           data.size() * sizeof(int));
    if (recv_size < 0) {
      printf("[警告] データが不正です: %d\n", recv_size);
      continue;
    }

    // recievefromIP = source.get_ip_address();
    // printf("Received %d bytes from %s\n", recv_size, recievefromIP);

    if (data[0] == -1) {
      printf("[警告] 緊急停止！\n");
      std::fill(data.begin() + 1, data.end(), 0); // 受信データを0で上書き
    }

    //方向成分と速度成分を分離
    for (int i = 1; i <= 6; i++) {
      if (data[i] >= 0) {
        mdd[i] = 1;
      } else {
        mdd[i] = 0;
      }
      mdp[i] = fabs((data[i]) / 100.0);
    }

    SERVO1.pulsewidth_us(map(data[7], 0, 270, 500, 2500));
    SERVO2.pulsewidth_us(map(data[8], 0, 270, 500, 2500));
    SERVO3.pulsewidth_us(map(data[9], 0, 270, 500, 2500));
    SERVO4.pulsewidth_us(map(data[10], 0, 270, 500, 2500));

    if (data[0] == 1 || data[0] == -1) {
      printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, "
             "%d, %d, %d\n",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6],
             data[7], data[8], data[9], data[10], data[11], data[12], data[13],
             data[14], data[15], data[16], data[17], data[18]);
    }

    // MDに出力
    MD1D = mdd[1];
    MD2D = mdd[2];
    MD3D = mdd[3];
    MD4D = mdd[4];
    MD5D = mdd[5];
    MD6D = mdd[6];

    MD1P = mdp[1];
    MD2P = mdp[2];
    MD3P = mdp[3];
    MD4P = mdp[4];
    MD5P = mdp[5];
    MD6P = mdp[6];

    // トランジスタに出力
    TR1 = data[11];
    TR2 = data[12];
    TR3 = data[13];
    TR4 = data[14];
    TR5 = data[15];
    TR6 = data[16];
    TR7 = data[17];
    TR8 = data[18];
  }
}