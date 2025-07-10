#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define MAX_ARRAY_SIZE 19

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

  //以下ピンの定義
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  if (received_data[0] == 1) {  //Bluetoothデバッグ
    SerialBT.print("Received: ");
    for (size_t i = 0; i < received_size; i++) {
      SerialBT.print(received_data[i]);
      SerialBT.print(", ");
    }
    SerialBT.println();
  }

  // 例：LEDを制御
  digitalWrite(5, received_data[2] == 1 ? HIGH : LOW);
  analogWrite(4,received_data[3]);


  //delay(10);
}
