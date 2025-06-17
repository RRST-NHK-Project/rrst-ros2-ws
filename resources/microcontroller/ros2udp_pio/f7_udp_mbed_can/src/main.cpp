/*
RRST NHK2025
IPアドレスは適宜変更
エンコーダーから計算した変位と速度をUDPで送信する
メイン基板V1.3
2025/03/18
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>
#include <vector>
#include "C610.hpp"

#define PI 3.141592653589793

//---------------------------QEI---------------------------//
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X4_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 2048, QEI::X4_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 2048, QEI::X4_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 2048, QEI::X4_ENCODING);
QEI ENC5(PE_8, PF_9, NC, 2048, QEI::X4_ENCODING);
QEI ENC6(PF_10, PD_11, NC, 2048, QEI::X4_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる
データシート: https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/

C610Array c610;

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
float Pulse[6]; // エンコーダーのパルス格納用
float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 速度の格納[mm/s]
float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 変位[m]

float period = 10; // 制御周期[ms]
float R = 80;      // オムニ直径[mm]
int PPRx4 = 8192;  // エンコーダーのResolution

double mdd[7]; // MDに出力する方向指令を格納
double mdp[7]; // MDに出力するduty比を格納

const char *recievefromIP = nullptr; //ネットワーク切断検知用

int main() {
   // 送信データ
   char sendData[32];
   for (int i = 0; i < 3; ++i) // 2台のモーターにIDを設定
      {
          c610[i].set_motor_id(i + 1);
      }

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

  // 送信先情報
  const char *destinationIP = "192.168.0.195";
  const uint16_t destinationPort = 4000;

  // 自機情報
  // const char *myIP = "192.168.8.215"; // MR_SD
  // const char *myIP = "192.168.0.216"; // MR
  // const char *myIP = "192.168.0.217"; // MD_SD
  const char *myIP = "192.168.0.218"; // DR
  //const char *myIP = "192.168.128.215"; // DR on test
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  //クラスのインスタンス化
  EthernetInterface net;
  SocketAddress destination, source, myData;
  UDPSocket udp;
  Thread receiveThread;

  //DHCPオフ（IPは固定）
  net.set_dhcp(false);
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error >_<\n");
    return -1;
  } else {
    printf("Network connection success ^_^\n");
    printf("IP: %s\n",myIP);
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
    using namespace std::chrono;

    // エンコーダーの値を取得
    Pulse[1] = float(ENC1.getPulses());
    Pulse[2] = float(ENC2.getPulses());
    Pulse[3] = float(ENC3.getPulses());
    Pulse[4] = float(ENC4.getPulses());

    v[1] = Pulse[1] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
    v[2] = Pulse[2] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
    v[3] = Pulse[3] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
    v[4] = Pulse[4] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算

    d[1] += Pulse[1] * R * PI / PPRx4 / 1000; //変位[m]
    d[2] += Pulse[2] * R * PI / PPRx4 / 1000; //変位[m]
    d[3] += Pulse[3] * R * PI / PPRx4 / 1000; //変位[m]
    d[4] += Pulse[4] * R * PI / PPRx4 / 1000; //変位[m]

    // エンコーダーをリセット
    ENC1.reset();
    ENC2.reset();
    ENC3.reset();
    ENC4.reset();

    // 速度データをカンマ区切りの文字列に変換
    char sendData[128]; // 送信データを格納する配列
    snprintf(sendData, sizeof(sendData), "%f,%f,%f,%f,%f,%f,%f,%f,", v[1], v[2],
             v[3], v[4], d[1], d[2], d[3], d[4]);

    //送信データを表示（デバッグ用）
    // printf("Sending (%d bytes): %s\n", strlen(sendData), sendData);

    // UDP送信
    if (const int result =
            udp.sendto(destination, sendData, strlen(sendData)) < 0) {
      printf("send Error: %d\n", result); // エラー処理
    }

    ThisThread::sleep_for(period); // 制御周期に合わせて待機
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

    int recv_size = receiver->recvfrom(&source, reinterpret_cast<char*>(data.data()), data.size() * sizeof(int));
    if (recv_size < 0) {
        printf("Receive Error: %d\n", recv_size);
        continue;
    }

    // recievefromIP = source.get_ip_address();
    // printf("Received %d bytes from %s\n", recv_size, recievefromIP);
   
    ////////////c610のコード///////////////
    auto now = Kernel::Clock::now();
    static auto pre = now;

    // CANメッセージを読み取り、C610オブジェクトにパース
    if (CANMessage msg; can.read(msg))
    {
        c610.parse_packet(msg);
    }

    // 10msごとに実行
    if (now - pre > 10ms)
    {
    int16_t current_0 = data[1] * 100; // data[6] の値をスケーリングして出力
    int16_t current_1 = data[2] * 100;           // モーター2用の電流値
    int16_t current_2 = data[3] * 100;          // モーター3用の電流値 
        c610[0].set_raw_current(current_0);
        c610[1].set_raw_current(current_1);
        c610[2].set_raw_current(current_2);
       // printf("data[6]: %d, darta[7]: %d, data[8]: %d\n",data[6],data[7],data[8]);
        //printf("Actual Current -> ID: %d, %d | ID: %d, %d | ID: %d, %d\n",
        c610[0].get_motor_id(), c610[0].get_rpm(),
        c610[1].get_motor_id(), c610[1].get_rpm(),
        c610[2].get_motor_id(), c610[2].get_rpm();
    // //printf("RPM: %d\n", c610[0].get_rpm());
        // エンコーダーの角度を計算 (度単位)
        //m2006のエンコーダーの現在のパルス*(10/8192) = 現在のangle

       //printf("%d\n",c610[0].get_rpm());
       // printf("%d", c610[0].get_absolute_angle()); // 先頭の C610 の角度を取得
       // printf("%d\n", c610[1].get_Angle());

        // 各モーターの電流を設定
        // for (auto &e : c610)
        // {
        //   //  e.set_raw_current(out);
        //     e.set_raw_current(current_0);
        //     e.set_raw_current(current_1);
        //     e.set_raw_current(current_2);
        // }

        // CANメッセージを生成して送信
        auto msg = c610.to_msgs();
        if (!can.write(msg))
        {
            printf("failed to write c610 msg\n");
        }

        pre = now;
    
}
///////ここまで///////

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

    if(data[0]){
      printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8],
        data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18]);
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
    TR8 = data[15];//臨時！！直せ！
  }
}