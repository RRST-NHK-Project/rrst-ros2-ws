#ifndef RCT_C610_HPP
#define RCT_C610_HPP

#include <mbed.h>
#include <array>

// C610モーターのデータを管理する構造体
struct C610
{
    int motor_id = 0; // モーターのID（デフォルト0）

    // モーターIDを設定
    void set_motor_id(int id) { motor_id = id; }
    // モーターIDを取得
    int get_motor_id() const { return motor_id; }

    // モーターの電流制限（-10000 〜 10000）
    static constexpr int16_t current_limit = 10000;
    static constexpr int16_t angle_max = 8192;
    static constexpr float angle_divisor = 146.5f;

    void set_current(float current)
    {
        raw_current_ = static_cast<int16_t>(current / 10.0f * current_limit);
    }

    void set_raw_current(int16_t raw_current)
    {
        raw_current_ = raw_current;
    }

    uint16_t get_angle() const { return rx_.angle; }
    int16_t get_rpm() const { return rx_.rpm; }
    float get_actual_current() const
    {
        return static_cast<float>(rx_.actual_current) / current_limit * 10.0f;
    }
    int16_t get_actual_raw_current() const { return rx_.actual_current; }
    uint8_t get_temp() const { return rx_.temp; }
    int16_t get_raw_current() const { return raw_current_; }

    void update_angle()
    {
        int16_t diff = rx_.angle - last_angle_;
        if (diff > angle_max / 2)
            diff -= angle_max;
        else if (diff < -angle_max / 2)
            diff += angle_max;
        angle_ += diff;
        last_angle_ = rx_.angle;
    }

    int32_t get_absolute_angle() const { return angle_; }

    float get_Angle() const
    {
        return static_cast<float>(angle_) / angle_divisor;
    }

    void parse_packet(const uint8_t data[8])
    {
        rx_.angle = uint16_t(data[0] << 8 | data[1]);
        rx_.rpm = int16_t(data[2] << 8 | data[3]);
        rx_.actual_current = int16_t(data[4] << 8 | data[5]);
        rx_.temp = data[6];

        update_angle();
    }

private:
    struct C610Packet
    {
        uint16_t angle;
        int16_t rpm;
        int16_t actual_current;
        uint8_t temp;
    } rx_ = {};

    int16_t raw_current_ = 0;
    int16_t last_angle_ = 0;
    int32_t angle_ = 0;
};

struct C610Array
{
    void parse_packet(const CANMessage &msg)
    {
        if (msg.format == CANStandard && msg.type == CANData &&
            msg.len == 8 && 0x201 <= msg.id && msg.id <= 0x208)
        {
            arr_[msg.id - 0x201].parse_packet(msg.data);
        }
    }

    auto to_msgs() -> CANMessage
    {
        uint8_t buf[8] = {};
        for (int i = 0; i < 4; ++i)
        {
            buf[i * 2] = static_cast<uint8_t>(arr_[i].get_raw_current() >> 8);
            buf[i * 2 + 1] = static_cast<uint8_t>(arr_[i].get_raw_current() & 0xFF);
        }
        return CANMessage{0x200, buf, 8};
    }

    auto &operator[](int index) { return arr_[index]; }
    auto begin() { return std::begin(arr_); }
    auto end() { return std::end(arr_); }

private:
    C610 arr_[8] = {};
};

#endif // RCT_C610_HPP