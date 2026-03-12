#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <memory>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class WT901CNode : public rclcpp::Node {
public:
    WT901CNode() : Node("wt901c_node") {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        open_serial("/dev/ttyUSB0");

        timer_ = create_wall_timer(
            5ms,
            std::bind(&WT901CNode::read_serial, this));
    }

private:
    int serial_fd_;

    void open_serial(const char *port) {
        serial_fd_ = open(port, O_RDWR | O_NOCTTY);

        struct termios tty{};
        tcgetattr(serial_fd_, &tty);

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tcsetattr(serial_fd_, TCSANOW, &tty);
    }

    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float roll = 0, pitch = 0, yaw = 0;

    void read_serial() {
        uint8_t buf[11];

        if (read(serial_fd_, buf, 11) != 11)
            return;

        if (buf[0] != 0x55)
            return;

        int16_t d1 = (buf[3] << 8) | buf[2];
        int16_t d2 = (buf[5] << 8) | buf[4];
        int16_t d3 = (buf[7] << 8) | buf[6];

        if (buf[1] == 0x51) {
            ax = d1 / 32768.0 * 16 * 9.8;
            ay = d2 / 32768.0 * 16 * 9.8;
            az = d3 / 32768.0 * 16 * 9.8;
        } else if (buf[1] == 0x52) {
            gx = d1 / 32768.0 * 2000 * M_PI / 180;
            gy = d2 / 32768.0 * 2000 * M_PI / 180;
            gz = d3 / 32768.0 * 2000 * M_PI / 180;
        } else if (buf[1] == 0x53) {
            roll = d1 / 32768.0 * M_PI;
            pitch = d2 / 32768.0 * M_PI;
            yaw = d3 / 32768.0 * M_PI;

            publish_imu();
        }
    }

    void publish_imu() {
        auto msg = sensor_msgs::msg::Imu();

        msg.header.stamp = now();
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        msg.orientation.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        msg.orientation.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        msg.orientation.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        msg.orientation.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

        imu_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WT901CNode>());
    rclcpp::shutdown();
}