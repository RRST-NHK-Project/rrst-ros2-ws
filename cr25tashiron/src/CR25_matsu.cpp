/*
2025, RRST-NHK-Project
ros2espパッケージ、マイコン側プログラム
microROSで受信したデータをもとにピン操作
ワイヤレスデバッグ(Bluetooth Serial)に対応。スマホアプリもしくはTeratermでデバッグ可能。
4MB版のESPでは容量が不足するためTools/PartitionSchemeからNO OTA(2MB APP/2MB SPIFFS)を選択すること。

TODO:時間経過でスタックするバグの修正
*/
#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <CAN.h>
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
// **複数のESPを使用する場合はIDを変更** //
#define ID 0
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

// microROS関連
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>

// ワイヤレスデバッグ
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  パルスカウンタ関連
#include "driver/pcnt.h"

// 受信配列の要素数を事前に定義
#define MAX_ARRAY_SIZE 19

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

// MD出力の上限値
#define MD_PWM_MAX 255

// ピンの定義 //
// エンコーダ
// #define ENC1_A 18
// #define ENC1_B 17
// #define ENC2_A 16
// #define ENC2_B 2
// #define ENC3_A 15
// #define ENC3_B 19
// #define ENC4_A 22
// #define ENC4_B 23
// MD PWM
#define MD1P 32
#define MD2P 33
#define MD3P 27
#define MD4P 14
// MD DIR
#define MD1D 25
#define MD2D 26
#define MD3D 12
#define MD4D 13

//---------定義--------//
const int ENCODER_MAX = 8192; //エンコーダの最大
const int HALF_ENCODER = ENCODER_MAX / 2;
constexpr float gear_ratio = 36.0f; // 減速比
constexpr int motor_id = 1; //モータID

// PWM周波数と分解能
const int pwm_freq = 1000;    // 1kHz
const int pwm_resolution = 8; // 8bit: 0〜255

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
// rcl_timer_t timer;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ノード名とトピック名の定義（ID付き）
String node_name = "esp32node_" + String(ID, DEC);
String publisher_topic_name = "from_esp32_" + String(ID, DEC);
String subscriber_topic_name = "to_esp32_" + String(ID, DEC);

// 受信データ格納用のバッファ
int32_t buffer[MAX_ARRAY_SIZE];

// 受信データ格納用
volatile int32_t received_data[MAX_ARRAY_SIZE]; // 受信データ
volatile size_t received_size = 0;              // 受信データのサイズ

// エンコーダのカウント格納用
int16_t count[4] = {0};

// #define RCCHECK(fn)             \
//     {                           \
//         if ((fn) != RCL_RET_OK) \
//             error_loop();       \
//     }

// 詳細デバッグ
#define RCCHECK(fn)                                                                     \
    do {                                                                                \
        rcl_ret_t temp_rc = fn;                                                         \
        if ((temp_rc) != RCL_RET_OK) {                                                  \
            SerialBT.printf("RCL error at %s:%d -> %d\n", __FILE__, __LINE__, temp_rc); \
            error_loop();                                                               \
        }                                                                               \
    } while (0)

// エラー発生時のループ
void error_loop() {
    while (1) {
        // このエラーが表示される場合は、シリアルモニターを閉じているか確認
        SerialBT.println("RCL Error!");
        delay(1000);
    }
}

// コールバック内でグローバル変数にコピー
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    size_t len = msg->data.size;
    if (len > MAX_ARRAY_SIZE)
        len = MAX_ARRAY_SIZE;

    for (size_t i = 0; i < len; i++) {
        received_data[i] = msg->data.data[i];
    }
    received_size = len;
}

//-----状態量-----//

int16_t encoder_count = 0;
int16_t rpm = 0;
int16_t last_encoder_count = -1;     // 前回の角度（0〜8191）
int32_t rotation_count = 0;     // 回転数（±）
int32_t total_encoder_count = 0; // 累積カウント（8192カウント/回転）
float angle = 0.0f; //出力軸角度
float vel = 0.0f; //出力速度
bool offset_ok = false;
int encoder_offset = 0;

int32_t current_position = 0; // 累積角度カウント
float motor_output_current_A = 0.0;
float limit = 19520;
float current_limit_A = 10.0f; // 最大出力電流（例：5A）

//------設定値-----//
float pos_setpoint = 720.0; //目標角度
float pos_input = 0.0; //現在のエンコーダの値
float pos_error_prev = 0.0;        // 前回の角度誤差
float pos_integral = 0.0;          // 角度積分項
float pos_output = 0;            // 角度PID出力（目標速度）

float vel_input = 0.0; //現在の速度
float vel_error_prev = 0.0;        // 前回の速度誤差
float vel_integral = 0.0;          // 速度積分項
float vel_output = 0;            // 速度PID出力

unsigned long lastPidTime = 0; // PID制御の時間計測用

//------------PIDゲイン-----------//
float kp_pos = 0.8;//0.4;
float ki_pos = 0.01;
float kd_pos = 0.02;//0.02;

// float kp_vel = 1.0;
// float ki_vel = 0.01;
// float kd_vel = 0.05;



//--------------関数作成-------------//
float pid(float setpoint, float input, float &error_prev, float &integral, float kp, float ki, float kd, float dt);
//setpoint:目標値,input:現在の値,&error_prev:前回の誤差,&integral:積分値
float constrain_double(float val, float min_val, float max_val);// 範囲制限

void send_cur(float cur) {
  constexpr float MAX_CUR = 10;
  constexpr int MAX_CUR_VAL = 10000;

  float val = cur * (MAX_CUR_VAL / MAX_CUR);
  if (val < -MAX_CUR_VAL) val = -MAX_CUR_VAL;
  else if (val > MAX_CUR_VAL) val = MAX_CUR_VAL;
  int16_t transmit_val = val;

  uint8_t send_data[8] = {};

  send_data[(motor_id - 1) * 2] = (transmit_val >> 8) & 0xFF;
  send_data[(motor_id - 1) * 2 + 1] = transmit_val & 0xFF;
  CAN.beginPacket(0x200);
  CAN.write(send_data, 8);
  CAN.endPacket();
}


void setup() {
    SerialBT.begin("ESP32_" + String(ID, DEC)); // Bluetoothの初期化
    delay(2000);

    // パルスカウンタの定義
    // プルアップを有効化
    // gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLUP_ONLY);

    // プルアップを有効化
    // gpio_set_pull_mode((gpio_num_t)ENC1_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC1_B, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC2_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC2_B, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC3_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC3_B, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC4_A, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode((gpio_num_t)ENC4_B, GPIO_PULLDOWN_ONLY);

    // パルスカウンタの設定
    pcnt_config_t pcnt_config1 = {};
    // pcnt_config1.pulse_gpio_num = ENC1_A;
    // pcnt_config1.ctrl_gpio_num = ENC1_B;
    pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config1.pos_mode = PCNT_COUNT_INC;
    pcnt_config1.neg_mode = PCNT_COUNT_DEC;
    pcnt_config1.counter_h_lim = COUNTER_H_LIM;
    pcnt_config1.counter_l_lim = COUNTER_L_LIM;
    pcnt_config1.unit = PCNT_UNIT_0;
    pcnt_config1.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config2 = {};
    // pcnt_config2.pulse_gpio_num = ENC1_B;
    // pcnt_config2.ctrl_gpio_num = ENC1_A;
    // pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config2.pos_mode = PCNT_COUNT_INC;
    // pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config2.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config2.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config2.unit = PCNT_UNIT_0;
    // pcnt_config2.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config3 = {};
    // pcnt_config3.pulse_gpio_num = ENC2_A;
    // pcnt_config3.ctrl_gpio_num = ENC2_B;
    pcnt_config3.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config3.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config3.pos_mode = PCNT_COUNT_INC;
    pcnt_config3.neg_mode = PCNT_COUNT_DEC;
    pcnt_config3.counter_h_lim = COUNTER_H_LIM;
    pcnt_config3.counter_l_lim = COUNTER_L_LIM;
    pcnt_config3.unit = PCNT_UNIT_1;
    pcnt_config3.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config4 = {};
    // pcnt_config4.pulse_gpio_num = ENC2_B;
    // pcnt_config4.ctrl_gpio_num = ENC2_A;
    // pcnt_config4.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config4.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config4.pos_mode = PCNT_COUNT_INC;
    // pcnt_config4.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config4.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config4.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config4.unit = PCNT_UNIT_1;
    // pcnt_config4.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config5 = {};
    // pcnt_config5.pulse_gpio_num = ENC3_A;
    // pcnt_config5.ctrl_gpio_num = ENC3_B;
    pcnt_config5.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config5.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config5.pos_mode = PCNT_COUNT_INC;
    pcnt_config5.neg_mode = PCNT_COUNT_DEC;
    pcnt_config5.counter_h_lim = COUNTER_H_LIM;
    pcnt_config5.counter_l_lim = COUNTER_L_LIM;
    pcnt_config5.unit = PCNT_UNIT_2;
    pcnt_config5.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config6 = {};
    // pcnt_config6.pulse_gpio_num = ENC3_B;
    // pcnt_config6.ctrl_gpio_num = ENC3_A;
    // pcnt_config6.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config6.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config6.pos_mode = PCNT_COUNT_INC;
    // pcnt_config6.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config6.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config6.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config6.unit = PCNT_UNIT_2;
    // pcnt_config6.channel = PCNT_CHANNEL_1;

    pcnt_config_t pcnt_config7 = {};
    pcnt_config7.pulse_gpio_num = ENC4_A;
    pcnt_config7.ctrl_gpio_num = ENC4_B;
    pcnt_config7.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config7.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config7.pos_mode = PCNT_COUNT_INC;
    pcnt_config7.neg_mode = PCNT_COUNT_DEC;
    pcnt_config7.counter_h_lim = COUNTER_H_LIM;
    pcnt_config7.counter_l_lim = COUNTER_L_LIM;
    pcnt_config7.unit = PCNT_UNIT_3;
    pcnt_config7.channel = PCNT_CHANNEL_0;

    // pcnt_config_t pcnt_config8 = {};
    // pcnt_config8.pulse_gpio_num = ENC4_B;
    // pcnt_config8.ctrl_gpio_num = ENC4_A;
    // pcnt_config8.lctrl_mode = PCNT_MODE_REVERSE;
    // pcnt_config8.hctrl_mode = PCNT_MODE_KEEP;
    // pcnt_config8.pos_mode = PCNT_COUNT_INC;
    // pcnt_config8.neg_mode = PCNT_COUNT_DEC;
    // pcnt_config8.counter_h_lim = COUNTER_H_LIM;
    // pcnt_config8.counter_l_lim = COUNTER_L_LIM;
    // pcnt_config8.unit = PCNT_UNIT_3;
    // pcnt_config8.channel = PCNT_CHANNEL_1;

    // パルスカウンタの初期化
    pcnt_unit_config(&pcnt_config1);
    // pcnt_unit_config(&pcnt_config2);
    pcnt_unit_config(&pcnt_config3);
    // pcnt_unit_config(&pcnt_config4);
    pcnt_unit_config(&pcnt_config5);
    // pcnt_unit_config(&pcnt_config6);
    pcnt_unit_config(&pcnt_config7);
    // pcnt_unit_config(&pcnt_config8);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_pause(PCNT_UNIT_2);
    pcnt_counter_pause(PCNT_UNIT_3);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2);
    pcnt_counter_clear(PCNT_UNIT_3);

    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_2);
    pcnt_counter_resume(PCNT_UNIT_3);

    // チャタrング防止のフィルターを有効化
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_filter_enable(PCNT_UNIT_2);
    pcnt_filter_enable(PCNT_UNIT_3);

    // フィルター値を設定
    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_2, PCNT_FILTER_VALUE);
    pcnt_set_filter_value(PCNT_UNIT_3, PCNT_FILTER_VALUE);

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    // Agentと接続できるまでリトライ
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        SerialBT.println("Waiting for agent...");
        delay(1000); // 1秒待つ
    }

    // Nodeの初期化
    RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

    // Subscriberの初期化
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        subscriber_topic_name.c_str()));

    // Publisherの初期化
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        publisher_topic_name.c_str()));

    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.data = buffer;
    msg.data.size = 0;
    msg.data.capacity = MAX_ARRAY_SIZE;

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 以下のサービスの数でexecutorのサイズを変える。

    // Executorにサービスを追加
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    // RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // ピンの初期化 //
    // MD DIR
    // pinMode(MD1P, OUTPUT);
    // pinMode(MD2P, OUTPUT);
    // pinMode(MD3P, OUTPUT);
    // pinMode(MD4P, OUTPUT);
    // // MD PWM
    // pinMode(MD1D, OUTPUT);
    // pinMode(MD2D, OUTPUT);
    // pinMode(MD3D, OUTPUT);
    // pinMode(MD4D, OUTPUT);
    ledcAttach(MD1P, pwm_freq, pwm_resolution);
    ledcAttach(MD2P, pwm_freq, pwm_resolution);
    ledcAttach(MD3P, pwm_freq, pwm_resolution);
    ledcAttach(MD4P, pwm_freq, pwm_resolution);

    // エンコーダ取得のスレッド（タスク）の作成
    xTaskCreateUniversal(
        ENC_Read_Task,
        "ENC_Read_Task",
        4096,
        NULL,
        2, // 優先度、最大25？
        NULL,
        APP_CPU_NUM);

    // 受信＆ピン操作のスレッド（タスク）の作成
    xTaskCreateUniversal(
        Pin_Output_Task,
        "Pin_Output_Task",
        4096,
        NULL,
        1, // 優先度、最大25？
        NULL,
        PRO_CPU_NUM);

    Serial.begin(115200);
  while (!Serial)
    ;

  CAN.setPins(25, 26);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

void ENC_Read_Task(void *pvParameters) {
    while (1) {
        // SerialBT.println("Reading encoders...");
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);
        pcnt_get_counter_value(PCNT_UNIT_2, &count[2]);
        pcnt_get_counter_value(PCNT_UNIT_3, &count[3]);

        // デバッグ用
        if (received_data[0] == 1) {
            SerialBT.printf("%d, %d, %d, %d\n", count[0], count[1], count[2], count[3]);
        }

        msg.data.size = 4;
        msg.data.data[0] = count[0];
        msg.data.data[1] = count[1];
        msg.data.data[2] = count[2];
        msg.data.data[3] = count[3];
        RCCHECK(rcl_publish(&publisher, &msg, NULL));

        vTaskDelay(1);
        // delay(1); // ウォッチドッグタイマのリセット(必須)
    }
}

void Pin_Output_Task(void *pvParameters) {
    while (1) {
        // 以下メインの処理

        // デバッグ用
        if (received_data[0] == 1) {
            SerialBT.print("Received: ");
            for (size_t i = 0; i < received_size; i++) {
                SerialBT.print(received_data[i]);
                SerialBT.print(", ");
            }
            SerialBT.println();
        }

        // MD出力の制限
        received_data[1] = constrain(received_data[1], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[2] = constrain(received_data[2], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[3] = constrain(received_data[3], -MD_PWM_MAX, MD_PWM_MAX);
        received_data[4] = constrain(received_data[4], -MD_PWM_MAX, MD_PWM_MAX);

        // ピンの操作
        digitalWrite(MD1D, received_data[1] > 0 ? HIGH : LOW);
        digitalWrite(MD2D, received_data[2] > 0 ? HIGH : LOW);
        digitalWrite(MD3D, received_data[3] > 0 ? HIGH : LOW);
        digitalWrite(MD4D, received_data[4] > 0 ? HIGH : LOW);

        // analogWrite(MD1P, abs(received_data[1]));
        // analogWrite(MD2P, abs(received_data[2]));
        // analogWrite(MD3P, abs(received_data[3]));
        // analogWrite(MD4P, abs(received_data[4]));

        ledcWrite(MD1P, abs(received_data[1]));
        ledcWrite(MD2P, abs(received_data[2]));
        ledcWrite(MD3P, abs(received_data[3]));
        ledcWrite(MD4P, abs(received_data[4]));

        vTaskDelay(1);
    }
}

loat pid(float setpoint, float input, float &error_prev, float &integral,
          float kp, float ki, float kd, float dt)
{
    float error = setpoint - input;
    integral += ((error + error_prev) * dt / 2.0f); // 台形積分
    float derivative = (error - error_prev) / dt;
    error_prev = error;

    return kp * error + ki * integral + kd * derivative;
}

float constrain_double(float val, float min_val, float max_val)
{
    if(val < min_val) return min_val;
    if(val > max_val) return max_val;
    return val;
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
    vTaskDelay(1);
    unsigned long now = millis();
    float dt = (now - lastPidTime) / 1000.0;
    if(dt <= 0) dt = 0.000001f; // dtが0にならないようにする
    lastPidTime = now;

  // 1. CAN受信
  int packetSize = CAN.parsePacket();
    while (packetSize) {  // 複数パケットも処理
        if (CAN.packetId() == 0x201) { // モータID=1
            uint8_t rx[8];
            for(int i=0;i<8;i++) rx[i] = CAN.read();
            encoder_count = (rx[0]<<8)|rx[1];
            rpm = (rx[2]<<8)|rx[3];

             // --- 初回オフセット設定 --- //
      if (!offset_ok) {
        encoder_offset = encoder_count;
        last_encoder_count = -1;
        rotation_count = 0;
        total_encoder_count = 0;
        pos_integral = 0;
        pos_error_prev = 0;
        offset_ok = true;
        Serial.println("Offset set!");
      }

      int enc_relative = encoder_count - encoder_offset;
      if (enc_relative < 0) enc_relative += ENCODER_MAX; // wrap-around補正

              if(last_encoder_count != -1) {
                int diff = encoder_count - last_encoder_count;
                if(diff > HALF_ENCODER) rotation_count--;
                else if(diff < -HALF_ENCODER) rotation_count++;
            }

            last_encoder_count = encoder_count;
            total_encoder_count = rotation_count * ENCODER_MAX + encoder_count;
            pos_input = total_encoder_count * (360.0 / (8192.0 * gear_ratio));
            vel_input = (rpm / gear_ratio) * 360.0 / 60.0;
        }
        packetSize = CAN.parsePacket(); // 次の受信も処理
    }
float pos_output = pid(pos_setpoint, pos_input, pos_error_prev, pos_integral, kp_pos, ki_pos, kd_pos, dt);
    //float vel_output = pid(pos_output, vel_input, vel_error_prev, vel_integral, kp_vel, ki_vel, kd_vel, dt);
    motor_output_current_A = constrain_double(pos_output, -current_limit_A, current_limit_A);
    //motor_output_current_A = 0.3;
  // 2. コマンド送信
  send_cur(motor_output_current_A);

  // 3. デバッグ出力
  //Serial.print("pos:\t"); Serial.println(pos_input);
  Serial.println(pos_setpoint-pos_input);

  delay(1);
}















/