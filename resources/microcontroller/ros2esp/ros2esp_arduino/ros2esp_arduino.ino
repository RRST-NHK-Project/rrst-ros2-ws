#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>

#define MAX_ARRAY_SIZE 64

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg;  // 送信用バッファ

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int32_t buffer[MAX_ARRAY_SIZE];

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// 受信コールバック
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg_in = (const std_msgs__msg__Int32MultiArray *)msgin;

  if (msg_in->data.size > 0) {
    int32_t led_state = msg_in->data.data[0];
    if (led_state == 1) {
      digitalWrite(5, HIGH);  // LED ON
    } else {
      digitalWrite(5, LOW);  // LED OFF
    }

    // 受信データをグローバルmsgにコピー（送信用）
    msg.data.size = msg_in->data.size;
    if (msg.data.size > MAX_ARRAY_SIZE) msg.data.size = MAX_ARRAY_SIZE;
    for (size_t i = 0; i < msg.data.size; i++) {
      msg.data.data[i] = msg_in->data.data[i];
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp_node_00", "", &support));

  // サブスクライバ初期化
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "mr_swerve_drive"));

  // パブリッシャ初期化（QoS指定不可環境）
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "esp32_debug_00"));

  // メッセージ初期化（送信用バッファ設定）
  std_msgs__msg__Int32MultiArray__init(&msg);
  msg.data.data = buffer;
  msg.data.capacity = MAX_ARRAY_SIZE;

  // 19個の0で初期化
  for (size_t i = 0; i < 19; i++) {
    buffer[i] = 0;
  }
  msg.data.size = 19;

  // Executor初期化
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // GPIO初期化
  pinMode(5, OUTPUT);
}

unsigned long last_publish_time = 0;
const unsigned long publish_interval = 100;

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  unsigned long now = millis();
  if (now - last_publish_time >= publish_interval) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    last_publish_time = now;
  }

  delay(10);
}
