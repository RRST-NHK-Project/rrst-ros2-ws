/*
ros2can (MODE_ROBOMAS) 経由でDJIロボマスの角度制御を行うホスト側テストプログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作 (common パッケージ)
#include "common/common.hpp"

// 以下マイコンに合わせて設定
// ros2can (xiao-esp32-s3_can2io, MODE_ROBOMAS) の DEVICE_ID / CAN_ID (指令送信先)
#define ROBOMAS_DEVICE_ID 101

// 制御対象のロボマスのインデックス (0-3: モータ1-4)
#define ROBOMAS_MOTOR_INDEX 0

// 位置フィードバック用AMTエンコーダを繋いだ別マイコンのDEVICE_ID (robomasとは別体)
#define ENCODER_DEVICE_ID 101

#define ARRAY_SIZE 24 // シリアルフレームのスロット数 (frame_data.hppのTx16NUM/Rx16NUMと同じ)

#define PUBLISH_RATE_MS 10 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

// 目標rpmの安全クランプ (念のための二重の飽和)
#define ROBOMAS_MAX_TARGET_RPM 300.0

// ===== 角度PDのゲイン =====
// 出力[rpm] = Kp * 偏差[deg] - Kd * 実速度[rpm]
// D項は角度の差分/dtでは作らない。受信が4ms間隔でバースト的に届くことがあり、
// dtの下限クランプ(5ms)と噛み合って微分が数倍に膨れ、±50rpmのバンバン発振に
// なるため(実測)。ファームが返す実測速度を使えば受信間隔と無関係になる。
// 重要: ファームの速度ループは ROBOMAS_KI_VEL = 0 (PD制御) なので、指令rpmは
// 速度指令ではなく実質的なトルク指令として効く。
//     電流[A] = ROBOMAS_KP_VEL * (指令rpm - 実速度rpm) = 0.02 * 偏差
// つまり 50rpm=1.0A / 100rpm=2.0A / 150rpm=3.0A(上限)。以下のrpm値はすべて
// 「どれだけトルクを要求するか」として読むこと。
//
// Kp: 偏差50degで上限に達する。
#define ANGLE_KP_RPM_PER_DEG 3.0
#define ANGLE_KD_RPM_PER_RPM 2.0
// 指令の上限 = トルクの上限。150rpm = 3.0Aで、GM6020の電流指令フレーム(0x1FE)の
// 仕様上限。これ以上はソフトでは出せない。
// 実測: 2.0A(100rpm)ではアームを持ち上げられず7秒以上停止したため上限まで上げた。
#define ANGLE_MAX_RPM 150.0

// ストール保護[s]。指令を出しているのに動かない状態がこの時間続いたら指令を切る。
// 3Aを流したまま停止しているとモータが焼けるため必須。復帰はボタン操作。
#define ANGLE_STALL_TIMEOUT_S 3.0

// 静止状態から動き出すのに必要な最低指令[rpm]。Ki=0なのでこれはトルクの下限を
// 意味する。150rpm = 3.0A(上限)。0.8A/1.6Aでは動き出せなかった。
#define ANGLE_MIN_TARGET_RPM 150.0
// 「止まっている」とみなす速度[rpm]。自重でずるずる流れている状態(実測3rpm程度)を
// 「止まっている」と誤判定して蹴らないよう、実測の流れ速度より小さく取る。
#define ANGLE_STOPPED_VEL_RPM 2.0
// 上記の速度が継続してこの時間[s]続いたときだけ底上げする。減速中に一瞬速度が
// 落ちただけで30rpmが立つのを防ぐ。
#define ANGLE_STOPPED_HOLD_S 0.2

// 目標とみなす許容誤差[deg]。この範囲に入ったら停止する。
#define ANGLE_TOLERANCE_DEG 2.0
// 停止後に再始動する偏差[deg]。停止判定のヒステリシス。許容誤差と同じ値にすると
// 境界で出入りを繰り返すので少し広げる。ボタン操作時は無条件に再始動する。
#define ANGLE_REARM_DEG 3.0

// 寸動(パルス駆動)モードに入る偏差[deg]。これより遠ければ通常のPDで速く寄せ、
// 近づいたらパルスで1回ずつ刻んで詰める。
#define ANGLE_FINE_RANGE_DEG 10.0
// パルスの最大ON時間[s]。動きを検出した時点で切るので、ここは「動かないときに
// あきらめるまでの時間」でしかない。
// ファームの速度PIはKp=0.005と小さく、偏差40rpmでもP項は0.2Aしか出ない。
// トルクは主に積分項が作るが、1A分育つのに 1/(0.05*40) = 0.5秒かかる。
// 0.3秒で切っていたときは積分が育つ前に指令が消え、実測255mA(ほぼP項のみ)で
// まったく蹴れていなかったため、積分が効くまで待てる長さにする。
#define ANGLE_PULSE_MAX_S 1.50
// パルスのOFF時間[s]。惰性が収まって角度が確定するのを待つ時間。短いと
// 前のパルスの余韻を見て次のパルスを打つので行き過ぎる。
#define ANGLE_PULSE_OFF_S 0.40
// 「動き出した」と判定する速度[rpm]。これを超えた瞬間にパルスを切る。
// 動き出してからも押し続けると行き過ぎるため、ここを小さくするほど細かく刻める。
#define ANGLE_MOVE_DETECT_RPM 3.0

// 目標到達後の保持用。指令0では電流も0になり自重でずり落ちるので、ずれた分だけ
// 指令を出し続ける。Ki=0なので指令がそのまま保持トルクの大きさになる
// (15rpm=0.3A, 40rpm=0.8A)。積分が育つのを待つ仕組みではない点に注意。
#define ANGLE_HOLD_KP_RPM_PER_DEG 10.0 // 保持中のPゲイン (偏差2degで20rpm=0.4A)
#define ANGLE_HOLD_MIN_RPM 15.0        // 保持に出し続ける最小指令 (0.3A)
#define ANGLE_HOLD_MAX_RPM 40.0        // 保持中の上限 (0.8A)。大きいと押し返しすぎる
#define ANGLE_HOLD_DEADBAND_DEG 0.3    // これ以内なら何も出さない

// 停止させたい場面での制動ゲイン [rpm指令 / rpm実速度]。
// 指令0は「0rpmを保て」の意味で制動ではあるが、ファームの速度ループのKpが
// 0.005と小さいため、偏差5rpmでも25mAしか出ず制動が弱い。実速度と逆向きの
// 指令を出して偏差を意図的に大きくすることで、強く止める。
#define ANGLE_BRAKE_GAIN 4.0
// 制動指令の上限[rpm]
#define ANGLE_MAX_BRAKE_RPM 50.0
// 指令が実際に効くまでの実効むだ時間[s] (publish周期+シリアル往復+ファーム周期)。
// この間に進む分を偏差から先読みで差し引くことで、D項の代わりの制動をかける。
#define ANGLE_LOOP_DELAY_S 0.03


class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl(uint8_t robomas_device_id, uint8_t encoder_device_id)
        : Node("hardware_control_" + std::to_string(robomas_device_id)),
          robomas_device_id_(robomas_device_id),
          encoder_device_id_(encoder_device_id)
    {
        /*
        ros2can (MODE_ROBOMAS) への指令フレーム (robomas.cpp 参照、24スロット):

        送信 (PC -> 本機, serial_tx_[ROBOMAS_DEVICE_ID]、本機側Rx_16Data):
          0-3: target_rpm (モータ1-4、生のrpm値、スケール無し)
          4-23: 未使用

        受信 (本機 -> PC, serial_rx_[ENCODER_DEVICE_ID]、本機側Tx_16Data):
          0-3: angle    [0.1deg単位] (出力軸換算、多回転累積済み)
          4-7: velocity [rpm]
          8-11: current [mA]
          12-23: 未使用

        位置フィードバックはrobomas自身の帰還 (data[0]) を使用するため、
        指令先とフィードバック元は同じマイコン (ROBOMAS_DEVICE_ID == ENCODER_DEVICE_ID)。
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // ros2can (robomas) へpublish (target_rpm指令)
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(robomas_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // AMTエンコーダ(robomasとは別体のマイコン)からのSubscribe
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(encoder_device_id_),
            10,
            std::bind(&HardWareControl::sensor_callback,
                      this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started (encoder: serial_rx_%d).",
                    robomas_device_id_, encoder_device_id_);
    }

private:
    // 目標を変えたときの処理。PDは偏差から毎回作り直すので内部状態のリセットは
    // 不要で、停止状態の解除だけ行う。
    void update_target()
    {
        holding_ = false; // ボタン操作は不感帯の中でも無条件に再始動させる
        prev_command_ = 0.0;
        stall_cutoff_ = false; // ストール保護の解除
        stall_since_ = now();  // 計測もやり直す
    }

    // 実速度と逆向きの指令を作る。止まっていれば0を返す。
    static double brake_command(double vel_rpm)
    {
        return std::clamp(-ANGLE_BRAKE_GAIN * vel_rpm, -ANGLE_MAX_BRAKE_RPM, ANGLE_MAX_BRAKE_RPM);
    }

    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        //bool L2 = msg->buttons[6];
        //bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;

        if (UP && !last_up_)
        {
            target_angle_deg_ += 90.0;
            update_target();
        }

        if (DOWN && !last_down_)
        {
            target_angle_deg_ -= 90.0;
            update_target();
        }

        // ×ボタンで0degへ復帰。ここでの0degはファーム起動時に
        // rotation_count=0 とした基準位置 (電源投入時にモータがいた回転内の
        // エンコーダ原点) であり、機構の原点とは限らない点に注意。
        if (CROSS && !last_cross_)
        {
            target_angle_deg_ = 0.0;
            update_target();
        }

        last_up_ = UP;
        last_down_ = DOWN;
        last_cross_ = CROSS;

        // vel: 実速度[rpm] (指令に追従できているか)
        // cur: 実電流[mA] (3000に張り付いていたらトルク飽和 = ソフトでは打つ手なし)
        RCLCPP_INFO(
            get_logger(),
            "target_angle= %.1f, angle= %.1f, target_rpm= %d, vel= %d, cur= %d",
            target_angle_deg_, enc1_total_angle_deg_, target_rpm_command_,
            vel_rpm_fb_, current_ma_fb_
        );
        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        msg.data.assign(ARRAY_SIZE, 0);
        msg.data[ROBOMAS_MOTOR_INDEX] = target_rpm_command_;

        publisher_->publish(msg);
    }

    void
    sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // ros2can (MODE_ROBOMAS) の帰還スロット0-3: 出力軸角度 [0.1deg単位]
        // GM6020はダイレクトドライブなのでギア比換算なし。ファーム側で多回転分を
        // 累積済みのため、ここでのラップ検出は不要。
        // int16のため±3276.7deg(約±9回転)を超えると折り返す点に注意。
        const int16_t angle_raw = msg->data[0];

        // 帰還スロット4-7: 出力軸速度 [rpm]。動き出しの底上げ判定に使う。
        const double vel_rpm = static_cast<double>(msg->data[4 + ROBOMAS_MOTOR_INDEX]);

        // ログ用 (帰還スロット8-11: 実電流 [mA])
        vel_rpm_fb_ = msg->data[4 + ROBOMAS_MOTOR_INDEX];
        current_ma_fb_ = msg->data[8 + ROBOMAS_MOTOR_INDEX];

        enc1_total_angle_deg_ = static_cast<double>(angle_raw) * 0.1;

        const rclcpp::Time current_time = now();

        if (!enc1_initialized_)
        {
            enc1_initialized_ = true;
            stopped_since_ = current_time;
            pulse_edge_time_ = current_time;
            stall_since_ = current_time;
        }

        if (!target_initialized_)
        {
            target_angle_deg_ = enc1_total_angle_deg_;
            update_target();
            target_initialized_ = true;
        }

        const double error_deg = target_angle_deg_ - enc1_total_angle_deg_;
        const double abs_error = std::abs(error_deg);

        // 角度PD。微分は実測速度から作るのでdtに依存しない。
        const double command = std::clamp(ANGLE_KP_RPM_PER_DEG * error_deg - ANGLE_KD_RPM_PER_RPM * vel_rpm,
                                          -ANGLE_MAX_RPM, ANGLE_MAX_RPM);

        // 指令のがたつきを抑える一次フィルタ
        const double smoothed_command = smoothing_alpha_ * prev_command_ + (1.0 - smoothing_alpha_) * command;
        prev_command_ = smoothed_command;

        // 停止判定 (ヒステリシス付き)。一度停止したらANGLE_REARM_DEGまで離れない
        // 限り再始動しない。同じ閾値で出入りさせると、境界の揺れで
        // 再始動->行き過ぎ->再始動を繰り返して止まらなくなる。
        if (holding_)
        {
            if (abs_error > ANGLE_REARM_DEG)
            {
                holding_ = false;
            }
        }
        else if (abs_error <= ANGLE_TOLERANCE_DEG)
        {
            holding_ = true;
        }

        // 「止まっている」状態が継続した時間を測る。動いている間は基準時刻を
        // 更新し続けるので、経過時間は0のままになる。
        if (std::abs(vel_rpm) >= ANGLE_STOPPED_VEL_RPM)
        {
            stopped_since_ = current_time;
        }
        const bool stalled = (current_time - stopped_since_).seconds() >= ANGLE_STOPPED_HOLD_S;

        double output_rpm = 0.0;

        if (holding_)
        {
            // 目標に入った。ここで指令0にすると力が抜けて自重で落ちるので、
            // ずれた分だけ小さな指令を出し続けてファーム側の積分に保持トルクを
            // 作らせる。上限を5rpmに抑えているので、積分が育っても大きくは動かない。
            if (abs_error > ANGLE_HOLD_DEADBAND_DEG)
            {
                const double hold = std::clamp(ANGLE_HOLD_KP_RPM_PER_DEG * abs_error,
                                               ANGLE_HOLD_MIN_RPM, ANGLE_HOLD_MAX_RPM);
                output_rpm = std::copysign(hold, error_deg);
            }
            else
            {
                output_rpm = brake_command(vel_rpm);
            }
            prev_command_ = 0.0; // フィルタに残った指令が尾を引かないようにする
            pulse_on_ = false;
            pulse_edge_time_ = current_time;
        }
        else if (abs_error > ANGLE_FINE_RANGE_DEG)
        {
            // 粗動: 通常のPD。ただし静止摩擦で動き出せない場合は底上げする。
            output_rpm = smoothed_command;
            if (stalled && std::abs(output_rpm) < ANGLE_MIN_TARGET_RPM)
            {
                output_rpm = std::copysign(ANGLE_MIN_TARGET_RPM, error_deg);
            }
            pulse_on_ = false;
            pulse_edge_time_ = current_time;
        }
        else
        {
            // 寸動: 動き出しに必要な40rpmを、動いた瞬間に切る短いパルスとして与える。
            // 押し続けると必ず行き過ぎるので、ONの終了条件は「時間」ではなく
            // 「動きを検出したこと」を主にする。OFF中に惰性が止まるのを待ってから
            // 次のパルスを打つので、1パルスあたりの移動量が機構の最小単位になる。
            const double elapsed = (current_time - pulse_edge_time_).seconds();

            if (pulse_on_)
            {
                if (std::abs(vel_rpm) >= ANGLE_MOVE_DETECT_RPM || elapsed >= ANGLE_PULSE_MAX_S)
                {
                    pulse_on_ = false;
                    pulse_edge_time_ = current_time;
                }
            }
            else if (elapsed >= ANGLE_PULSE_OFF_S)
            {
                pulse_on_ = true;
                pulse_edge_time_ = current_time;
            }

            // OFF中は惰性を待つのではなく積極的に止める。ここで止めきれた分だけ
            // 1パルスあたりの移動量が小さくなり、細かく詰められる。
            output_rpm = pulse_on_ ? std::copysign(ANGLE_MIN_TARGET_RPM, error_deg)
                                   : brake_command(vel_rpm);
            prev_command_ = 0.0; // PDのフィルタ状態を寸動に持ち越さない
        }

        // ストール保護: 指令を出しているのに動かない状態が続いたら指令を切る。
        // 最大電流を流したまま停止し続けるとモータが焼けるため。
        // 復帰はボタン操作 (update_target) のみ。
        if (stall_cutoff_)
        {
            output_rpm = 0.0;
            // 保護中も計測を進めると、解除した瞬間に経過時間が閾値を超えたまま
            // 残っていて即座に再発火するのでリセットし続ける。
            stall_since_ = current_time;
        }
        else if (std::abs(output_rpm) < 1.0 || std::abs(vel_rpm) >= ANGLE_STOPPED_VEL_RPM)
        {
            stall_since_ = current_time; // 指令を出していない、または動いている
        }
        else if ((current_time - stall_since_).seconds() >= ANGLE_STALL_TIMEOUT_S)
        {
            stall_cutoff_ = true;
            RCLCPP_WARN(get_logger(),
                        "STALL: %.1f秒間 指令%.0frpmに対して動きなし。指令を切りました。"
                        "ボタン操作で復帰します。",
                        ANGLE_STALL_TIMEOUT_S, output_rpm);
            output_rpm = 0.0;
        }

        // 指令はint16(1rpm刻み)なので四捨五入で丸める
        const double clamped_command =
            std::clamp(output_rpm, -ROBOMAS_MAX_TARGET_RPM, ROBOMAS_MAX_TARGET_RPM);
        target_rpm_command_ = static_cast<int16_t>(std::lround(clamped_command));

        // const double error_deg = target_angle_deg_ - angle_deg;
        // RCLCPP_INFO(get_logger(), "angle: %.3f deg target: %.3f deg err: %.3f cmd: %.3f smoothed: %.3f target_rpm: %d",
        //         angle_deg, target_angle_deg_, error_deg, static_cast<double>(command), smoothed_command, target_rpm_command_);
        // 受信データ処理ここまで
    }

    uint8_t robomas_device_id_;
    uint8_t encoder_device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool enc1_initialized_ = false;
    double enc1_total_angle_deg_ = 0.0;
    bool last_up_ = false;
    bool last_down_ =false;
    bool last_cross_ = false;
    bool target_initialized_ = false;
    double target_angle_deg_ = 0.0;
    rclcpp::Time stopped_since_;
    bool holding_ = false;
    // ストール保護
    rclcpp::Time stall_since_;
    bool stall_cutoff_ = false;
    // 寸動モードの状態
    rclcpp::Time pulse_edge_time_;
    bool pulse_on_ = false;

    // 指令の一次フィルタ (大きいほど平滑だが遅れる)
    double prev_command_ = 0.0;
    const double smoothing_alpha_ = 0.5;

    int16_t target_rpm_command_ = 0;
    int16_t vel_rpm_fb_ = 0;
    int16_t current_ma_fb_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet ! Welcome Day !";
    int result = std::system(figletout.c_str());
    if (result != 0)
    {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    auto hardware_control = std::make_shared<HardWareControl>(ROBOMAS_DEVICE_ID, ENCODER_DEVICE_ID);
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
