#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>

#define MAX_ARRAY_SIZE 64

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;

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

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  if (msg->data.size > 0) {
    //受信後の処理はここから！
    int32_t led_state = msg->data.data[0];
    if (led_state == 1) {
      digitalWrite(5, HIGH);  // LED ON
    } else {
      digitalWrite(5, LOW);  // LED OFF
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

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "mr_swerve_drive"));  //サブスクライブするトピック名

  std_msgs__msg__Int32MultiArray__init(&msg);
  msg.data.data = buffer;
  msg.data.size = 0;
  msg.data.capacity = MAX_ARRAY_SIZE;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  //以下ピンの定義
  pinMode(5,OUTPUT);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
