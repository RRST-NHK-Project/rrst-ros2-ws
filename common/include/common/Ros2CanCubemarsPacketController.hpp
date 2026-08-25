// マイコン(ros2can, MODE_CUBEMARS)へ送受信する配列tx_/rx_へ安全にアクセスするための
// ライブラリ。汎用IOノード向け Ros2CanPacketController.hpp の CubeMars 版。
//
// MODE_CUBEMARS (xiao-esp32-s3_can2io) は MODE_CAN_HOST と違い、24スロットを
// CANバス上のノードへ分配しない「独立デバイス」として動作する。24スロットをそのまま
// CubeMars AKシリーズ (AK40-10等、Servo(CAN)モード、1Mbps固定) 最大 MOTOR_COUNT 台分の
// 指令/帰還に直接割り当てる (スロット0起点、ノードオフセット無し)。
// [firmware/xiao-esp32-s3_can2io/src/cubemars.cpp のスロット割当、および
//  ros2can/ros2can/device_profiles.py の make_cubemars_profile() と一致させること]
//
// 速度/位置ともアクチュエータ内蔵のクローズドループが指令にそのまま追従するため、
// ロボマスのGM6020のようなホスト側PID (common/PID.hpp) は不要。
//
// 指令 (TX, PC -> マイコン -> CAN -> 各モータ):
//   0-3:   target (モータ1-4、意味は control_mode に依存)
//          control_mode=MODE_VELOCITY : 電気角速度、10ERPM/LSB (±327670ERPM)
//          control_mode=MODE_POSITION : 位置、0.1deg/LSB (±3276.7deg)
//          control_mode=MODE_MIT      : 目標位置、0.1deg/LSB (setMit()の範囲でクランプ)
//   4-7:   control_mode (モータ1-4): 0=速度ループ, 1=位置ループ, 2=MIT(Force Control)
//   8-11:  MIT用 目標速度 (モータ1-4)、0.01rad/s/LSB (control_mode=MODE_MITのみ参照)
//   12-15: MIT用 Kp (モータ1-4)、0.1/LSB、レンジ0-500 (control_mode=MODE_MITのみ参照)
//   16-19: MIT用 Kd (モータ1-4)、0.01/LSB、レンジ0-5 (control_mode=MODE_MITのみ参照)
//   20-23: MIT用 目標トルクFF (モータ1-4)、0.01N・m/LSB (control_mode=MODE_MITのみ参照)
//
// MIT(Force Control)モード (AK Series Module Product Manual V3.2.0 4.2節) は
// 位置・速度・Kp・Kd・トルクFFを同時に指令し、モータ側で
// torque = Kp*(pos_des-pos) + Kd*(vel_des-vel) + torque_ff を計算するインピーダンス
// 制御。CAN ID/フレーム形式はServo(CAN)モード(速度/位置ループ)と共通の拡張ID方式
// なので、cubemars.cppはcontrol_modeスロットの値でモータごとに毎周期選択するだけでよい。
//
// MIT_P/V/T_MIN~MAX (setMit()が使うクランプ範囲) は、マニュアルのパラメータ表に
// AK40-10の掲載が無いため他モータからの類推値。ファーム側config.hppの
// CUBEMARS_MIT_*と必ず一致させ、実機のR-Link(CubeMarsTool)設定と揃えること。
// 不一致だと指令値の意味(rad, rad/s, N・m)がズレて意図しない速度・トルクが出力される。
//
// 帰還 (RX, 各モータ -> CAN -> マイコン -> PC。マニュアル4.3.1のCAN Upload Message
//       Protocol (Function ID 0x29) の値をスケール変換無しでそのまま格納):
//   0-3:   position    [0.1deg/LSB]
//   4-7:   speed       [10ERPM/LSB] (電気角速度。出力軸rpmへの換算は極対数・減速比が
//                                    モータ機種依存のため本ライブラリでは行わない)
//   8-11:  current     [0.01A/LSB]
//   12-15: temperature [degC]
//   16-19: error code  (ErrorCode 参照)
//   20-23: 未使用
//
// 安全性について:
//   tx_ の初期値は全ゼロ = control_mode 0(速度) かつ target 0 なので、起動直後や
//   E-STOP時は「ゼロ速度指令(その場停止)」になり位置ジャンプは発生しない。
//   逆に setPosition() を呼んだ瞬間に現在位置から指定位置へ移動を開始するため、
//   最初の位置指令は現在位置から離れすぎていないか注意すること。
//   ノード終了時は stopAll() を呼んで全モータをゼロ速度に戻すのが望ましい
//   (firmware側にCAN/シリアル途絶時のフェイルセーフは無く、最後の指令を保持し続ける)。
//
// 各モータのCAN IDはファーム側 config.hpp の CUBEMARS_MOTOR_ID_n でコンパイル時固定
// (R-Link CubeMarsTool の設定と一致させること)。canId() はその既定値を返す。

// ===== 使用例コメント =====
/*
// 変数宣言 (Ros2CanCubemarsPacketController のインスタンス)
Ros2CanCubemarsPacketController ctrlPkt;

// モータ1(0-origin: 0)を位置ループで90degへ
ctrlPkt.setPosition(0, 90.0);

// モータ2(0-origin: 1)を速度ループで3000ERPMへ
ctrlPkt.setVelocity(1, 3000.0);

// モータ3(0-origin: 2)をMIT(Force Control)モードで、90degへKp=20,Kd=1.0で追従
ctrlPkt.setMit(2, 90.0, 0.0, 20.0, 1.0, 0.0);

// 送信用配列を取得してpublish
auto sendArray = ctrlPkt.toVector();

// 受信したInt16MultiArrayでrx_を更新
ctrlPkt.updateRx(msg->data);

// モータ1の現在位置[deg]、電流[A]、エラーを取得
double pos = ctrlPkt.getPosition(0);
double cur = ctrlPkt.getCurrent(0);
const char *err = Ros2CanCubemarsPacketController::errorText(ctrlPkt.getError(0));
*/

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

class Ros2CanCubemarsPacketController {
public:
    // ファームウェア既定値 (config.hpp の CUBEMARS_MOTOR_COUNT)
    static constexpr int MOTOR_COUNT = 4;
    static constexpr int DATA_SIZE = 24; // serial_bridge互換フレームのスロット数(固定)

    // スロットのスケール (cubemars.cpp と一致させること)
    static constexpr double POSITION_LSB_DEG = 0.1;    // 位置 0.1deg/LSB
    static constexpr double VELOCITY_LSB_ERPM = 10.0;  // 電気角速度 10ERPM/LSB
    static constexpr double CURRENT_LSB_A = 0.01;      // 電流 0.01A/LSB

    // 指令の有効範囲 (int16スロット x 上記スケール)
    static constexpr double POSITION_MIN_DEG = -3276.8;
    static constexpr double POSITION_MAX_DEG = 3276.7;
    static constexpr double VELOCITY_MIN_ERPM = -327680.0;
    static constexpr double VELOCITY_MAX_ERPM = 327670.0;

    // MITモード用スロットのスケール (cubemars.cpp と一致させること)
    static constexpr double MIT_VELOCITY_LSB_RADPS = 0.01; // 目標速度 0.01rad/s/LSB
    static constexpr double MIT_KP_LSB = 0.1;               // Kp 0.1/LSB
    static constexpr double MIT_KD_LSB = 0.01;              // Kd 0.01/LSB
    static constexpr double MIT_TORQUE_LSB_NM = 0.01;       // トルクFF 0.01N・m/LSB

    // MITモードのクランプ範囲 (setMit()が使用)。ファーム側config.hppの
    // CUBEMARS_MIT_*、および実機のR-Link設定と必ず一致させること
    // (不一致だと指令値の意味がズレる。ヘッダ冒頭コメント参照)。
    static constexpr double MIT_POSITION_MIN_DEG = -716.2; // ≈ -12.5rad
    static constexpr double MIT_POSITION_MAX_DEG = 716.2;  // ≈ 12.5rad
    static constexpr double MIT_VELOCITY_MIN_RADPS = -50.0;
    static constexpr double MIT_VELOCITY_MAX_RADPS = 50.0;
    static constexpr double MIT_TORQUE_MIN_NM = -18.0;
    static constexpr double MIT_TORQUE_MAX_NM = 18.0;
    static constexpr double MIT_KP_MIN = 0.0;
    static constexpr double MIT_KP_MAX = 500.0;
    static constexpr double MIT_KD_MIN = 0.0;
    static constexpr double MIT_KD_MAX = 5.0;

    // control_mode スロットの値 (cubemars.cpp の CUBEMARS_MODE_* と一致させること)
    enum ControlMode : int16_t {
        MODE_VELOCITY = 0, // 速度ループ (target = 電気角速度)
        MODE_POSITION = 1, // 位置ループ (target = 位置)
        MODE_MIT = 2,      // MIT(Force Control)。位置+速度+Kp+Kd+トルクFFを同時指令
    };

    // error code スロットの値 (マニュアル4.3.1)
    enum ErrorCode : int16_t {
        ERR_NONE = 0,
        ERR_MOTOR_OVER_TEMP = 1,
        ERR_OVER_CURRENT = 2,
        ERR_OVER_VOLTAGE = 3,
        ERR_UNDER_VOLTAGE = 4,
        ERR_ENCODER_FAULT = 5,
        ERR_MOSFET_OVER_TEMP = 6,
        ERR_MOTOR_STALL = 7,
    };

    // 各データのスロット先頭index (モータm(0-origin)は base + m)
    enum TxSlot : int {
        SLOT_TARGET = 0,        // 0-3
        SLOT_CONTROL_MODE = 4,  // 4-7
        SLOT_MIT_VELOCITY = 8,  // 8-11  (control_mode=MODE_MITのみ参照)
        SLOT_MIT_KP = 12,       // 12-15 (control_mode=MODE_MITのみ参照)
        SLOT_MIT_KD = 16,       // 16-19 (control_mode=MODE_MITのみ参照)
        SLOT_MIT_TORQUE_FF = 20, // 20-23 (control_mode=MODE_MITのみ参照)
    };
    enum RxSlot : int {
        SLOT_POSITION = 0,     // 0-3
        SLOT_SPEED = 4,        // 4-7
        SLOT_CURRENT = 8,      // 8-11
        SLOT_TEMPERATURE = 12, // 12-15
        SLOT_ERROR = 16,       // 16-19
    };

    // 送信配列本体 (ROS -> マイコン -> CAN -> 各モータ)
    std::array<int16_t, DATA_SIZE> tx_{};
    // 受信配列本体 (各モータ -> CAN -> マイコン -> ROS)
    std::array<int16_t, DATA_SIZE> rx_{};

    // ---- 指令 (TX) ----

    // 指定モータ(0-origin, 0~MOTOR_COUNT-1)を位置ループで角度deg[±3276.7]へ動かす
    // (範囲外degはクランプ、motor範囲外は無視)。
    // 呼んだ瞬間に現在位置から目標位置へ移動を開始するので、初回の指令値に注意。
    void setPosition(int motor, double deg) {
        if (!validMotor(motor))
            return;
        tx_[SLOT_CONTROL_MODE + motor] = MODE_POSITION;
        tx_[SLOT_TARGET + motor] =
            toSlot(std::clamp(deg, POSITION_MIN_DEG, POSITION_MAX_DEG) / POSITION_LSB_DEG);
    }

    // 指定モータを速度ループで電気角速度erpm[±327670]で回す
    // (範囲外erpmはクランプ、motor範囲外は無視)。
    // 出力軸rpmではなく電気角速度なので注意 (換算比は極対数・減速比によりモータ機種依存)。
    void setVelocity(int motor, double erpm) {
        if (!validMotor(motor))
            return;
        tx_[SLOT_CONTROL_MODE + motor] = MODE_VELOCITY;
        tx_[SLOT_TARGET + motor] =
            toSlot(std::clamp(erpm, VELOCITY_MIN_ERPM, VELOCITY_MAX_ERPM) / VELOCITY_LSB_ERPM);
    }

    // 指定モータをMIT(Force Control)モードで駆動する。位置(deg)・速度(rad/s)・
    // Kp・Kd・トルクFF(N・m)を同時に指令し、モータ側で
    // torque = Kp*(pos_des-pos) + Kd*(vel_des-vel) + torque_ff を計算する
    // インピーダンス制御 (マニュアル4.2節)。各値はMIT_*_MIN~MAXへクランプされる
    // (motor範囲外は無視)。この範囲がファーム側config.hppのCUBEMARS_MIT_*と
    // 一致していないと指令値の意味がズレるので、ヘッダ冒頭コメントも参照のこと。
    // 呼んだ瞬間に現在値からの追従を開始するため、初回はKp/Kdを小さめに設定して
    // 挙動を確認してから上げること。
    void setMit(int motor, double pos_deg, double vel_radps, double kp, double kd, double torque_ff_nm) {
        if (!validMotor(motor))
            return;
        tx_[SLOT_CONTROL_MODE + motor] = MODE_MIT;
        tx_[SLOT_TARGET + motor] =
            toSlot(std::clamp(pos_deg, MIT_POSITION_MIN_DEG, MIT_POSITION_MAX_DEG) / POSITION_LSB_DEG);
        tx_[SLOT_MIT_VELOCITY + motor] =
            toSlot(std::clamp(vel_radps, MIT_VELOCITY_MIN_RADPS, MIT_VELOCITY_MAX_RADPS) / MIT_VELOCITY_LSB_RADPS);
        tx_[SLOT_MIT_KP + motor] =
            toSlot(std::clamp(kp, MIT_KP_MIN, MIT_KP_MAX) / MIT_KP_LSB);
        tx_[SLOT_MIT_KD + motor] =
            toSlot(std::clamp(kd, MIT_KD_MIN, MIT_KD_MAX) / MIT_KD_LSB);
        tx_[SLOT_MIT_TORQUE_FF + motor] =
            toSlot(std::clamp(torque_ff_nm, MIT_TORQUE_MIN_NM, MIT_TORQUE_MAX_NM) / MIT_TORQUE_LSB_NM);
    }

    // 指定モータをゼロ速度指令(その場停止)にする。位置ジャンプは発生しない。
    void stop(int motor) {
        if (!validMotor(motor))
            return;
        tx_[SLOT_CONTROL_MODE + motor] = MODE_VELOCITY;
        tx_[SLOT_TARGET + motor] = 0;
    }

    // 全モータをゼロ速度指令にする (ノード終了時のフェイルセーフ用)
    void stopAll() {
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            stop(m);
        }
    }

    // 指定モータに現在設定されているcontrol_modeを取得する
    ControlMode getControlMode(int motor) const {
        if (!validMotor(motor))
            return MODE_VELOCITY;
        int16_t mode = tx_[SLOT_CONTROL_MODE + motor];
        if (mode == MODE_POSITION)
            return MODE_POSITION;
        if (mode == MODE_MIT)
            return MODE_MIT;
        return MODE_VELOCITY;
    }

    // ---- 帰還 (RX) ----

    // 指定モータの現在位置[deg]を取得する (motor範囲外は0を返す)
    double getPosition(int motor) const {
        return validMotor(motor) ? rx_[SLOT_POSITION + motor] * POSITION_LSB_DEG : 0.0;
    }

    // 指定モータの現在速度[電気角ERPM]を取得する (motor範囲外は0を返す)
    double getVelocity(int motor) const {
        return validMotor(motor) ? rx_[SLOT_SPEED + motor] * VELOCITY_LSB_ERPM : 0.0;
    }

    // 指定モータの実測電流[A]を取得する (motor範囲外は0を返す)
    double getCurrent(int motor) const {
        return validMotor(motor) ? rx_[SLOT_CURRENT + motor] * CURRENT_LSB_A : 0.0;
    }

    // 指定モータの温度[degC]を取得する (motor範囲外は0を返す)
    int getTemperature(int motor) const {
        return validMotor(motor) ? static_cast<int>(rx_[SLOT_TEMPERATURE + motor]) : 0;
    }

    // 指定モータのエラーコードを取得する (motor範囲外はERR_NONEを返す)
    int16_t getError(int motor) const {
        return validMotor(motor) ? rx_[SLOT_ERROR + motor] : static_cast<int16_t>(ERR_NONE);
    }

    // エラーコードを人が読める文字列にする (ログ出力用)
    static const char *errorText(int16_t code) {
        switch (code) {
        case ERR_NONE:
            return "no fault";
        case ERR_MOTOR_OVER_TEMP:
            return "motor over-temp";
        case ERR_OVER_CURRENT:
            return "over-current";
        case ERR_OVER_VOLTAGE:
            return "over-voltage";
        case ERR_UNDER_VOLTAGE:
            return "under-voltage";
        case ERR_ENCODER_FAULT:
            return "encoder fault";
        case ERR_MOSFET_OVER_TEMP:
            return "MOSFET over-temp";
        case ERR_MOTOR_STALL:
            return "motor stall";
        default:
            return "unknown";
        }
    }

    // ---- 配列の入出力 ----

    // motor(0-origin)に対応するCAN_IDの既定値 (101,102,103,104)。
    // ファーム側 config.hpp の CUBEMARS_MOTOR_ID_n を変更した場合は一致しなくなる。
    static int canId(int motor) { return 101 + motor; }

    // 受信したInt16MultiArray相当のdataでrx_を更新
    void updateRx(const std::vector<int16_t> &data) {
        for (size_t i = 0; i < DATA_SIZE && i < data.size(); ++i) {
            rx_[i] = data[i];
        }
    }

    // 送信用配列を vector で取得
    std::vector<int16_t> toVector() const {
        return std::vector<int16_t>(tx_.begin(), tx_.end());
    }

    // 直接アクセスも残す (TX配列、グローバルslot index指定)
    int16_t &operator[](int index) { return tx_[index]; }
    const int16_t &operator[](int index) const { return tx_[index]; }

private:
    static bool validMotor(int motor) { return motor >= 0 && motor < MOTOR_COUNT; }

    // スケール済みの実数値をint16スロット値へ丸める
    static int16_t toSlot(double scaled) {
        return static_cast<int16_t>(std::lround(std::clamp(scaled, -32768.0, 32767.0)));
    }
};
