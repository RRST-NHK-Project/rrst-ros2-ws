/*
2025, RRST-NHK-Project
ros2espパッケージ、マイコン側プログラム
microROSで受信したデータをもとにピン操作
ワイヤレスデバッグ(Bluetooth Serial)に対応しています。スマホアプリもしくはTeratermでデバッグ可能です。現時点では時間経過でスタックするバグがあります。ROS側からオンオフ切り替えできます。
４MB版のESPでは容量が不足するためTools/PartitionSchemeからNO OTA(2MB APP/2MB SPIFFS)を選択してください。
*/

//microROS関連
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

//ワイヤレスデバッグで使う
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define MAX_ARRAY_SIZE 19

// ピンの定義 //

//通信状態表示
#define LED 2

// #define MD1P PA_0
// #define MD2P PA_3
// #define MD3P PC_7
// #define MD4P PC_6
// #define MD5P PC_8
// #define MD6P PC_9

// // MD DIR
// #define MD1D PD_2
// #define MD2D PG_2
// #define MD3D PD_5
// #define MD4D PD_6
// #define MD5D PD_7
// #define MD6D PC_10

// // サーボ
// #define SERVO1 PB_1
// #define SERVO2 PB_6
// #define SERVO3 PD_13
// #define SERVO4 PB_8

// // トランジスタ（ソレノイド・表示灯）
// #define TR1 PF_0
// #define TR2 PF_1
// #define TR3 PF_15
// #define TR4 PC_11
// #define TR5 PC_12
// #define TR6 PF_14
// #define TR7 PF_12
// #define TR8 PF_13

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
int32_t buffer[MAX_ARRAY_SIZE];

// ★ 受信データ格納用（グローバル）
volatile int32_t received_data[MAX_ARRAY_SIZE];  // 最新の受信データ
volatile size_t received_size = 0;               // 受信データのサイズ

#define RCCHECK(fn) \
  { \
    if ((fn) != RCL_RET_OK) error_loop(); \
  }

void error_loop() {
  while (1) {
    SerialBT.println("RCL Error!");
    delay(1000);
  }
}

// ★ コールバック内でグローバル変数にコピー
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  size_t len = msg->data.size;
  if (len > MAX_ARRAY_SIZE) len = MAX_ARRAY_SIZE;

  for (size_t i = 0; i < len; i++) {
    received_data[i] = msg->data.data[i];
  }

  received_size = len;
}

void setup() {
  SerialBT.begin("ESP32");
  delay(2000);

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  //通信状態表示LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // ★ エージェントと接続できるまでリトライ
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    SerialBT.println("Waiting for agent...");
    delay(1000);  // 1秒待つ
  }

  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp_node_00", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "mr_swerve_drive"));

  std_msgs__msg__Int32MultiArray__init(&msg);
  msg.data.data = buffer;
  msg.data.size = 0;
  msg.data.capacity = MAX_ARRAY_SIZE;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // ピンの初期化 //

  // MD DIR
  // pinMode(MD1P, OUTPUT);
  // pinMode(MD2P, OUTPUT);
  // pinMode(MD3P, OUTPUT);
  // pinMode(MD4P, OUTPUT);
  // pinMode(MD5P, OUTPUT);
  // pinMode(MD6P, OUTPUT);

  // // MD PWM
  // pinMode(MD1D, OUTPUT);
  // pinMode(MD2D, OUTPUT);
  // pinMode(MD3D, OUTPUT);
  // pinMode(MD4D, OUTPUT);
  // pinMode(MD5D, OUTPUT);
  // pinMode(MD6D, OUTPUT);

  // // サーボ
  // pinMode(SERVO1, OUTPUT);
  // pinMode(SERVO2, OUTPUT);
  // pinMode(SERVO3, OUTPUT);
  // pinMode(SERVO4, OUTPUT);

  // // トランジスタ(ソレノイド・表示灯)
  // pinMode(TR1, OUTPUT);
  // pinMode(TR2, OUTPUT);
  // pinMode(TR3, OUTPUT);
  // pinMode(TR4, OUTPUT);
  // pinMode(TR5, OUTPUT);
  // pinMode(TR6, OUTPUT);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  digitalWrite(LED, HIGH);

  //以下メインの処理

  //デバッグ用
  if (received_data[0] == 1) {
    SerialBT.print("Received: ");
    for (size_t i = 0; i < received_size; i++) {
      SerialBT.print(received_data[i]);
      SerialBT.print(", ");
    }
    SerialBT.println();
  }

  //ピンの操作

  //ここまで

  //delay(10);
}
